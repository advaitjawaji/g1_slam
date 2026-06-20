"""G1 odom bridge.

Publishes ``/odom`` (``nav_msgs/Odometry``) and the ``odom -> pelvis`` TF so
Nav2 can localize the robot **without a camera / SLAM source** (Tier 3).

Odometry source is selected automatically, best first:

1. **SportModeState** (``rt/sportmodestate``, ``unitree_go`` type) — full world
   pose (position, velocity, yaw). Whether the G1 firmware publishes this is
   firmware-dependent; if no messages arrive we fall through. Topic is a param
   (some firmware uses ``rt/odommodestate``).
2. **IMU yaw + cmd_vel** — the G1 *always* publishes ``rt/lowstate``
   (``unitree_hg``) carrying an IMU with absolute ``rpy`` yaw. We take true yaw
   from the IMU and integrate XY from the latest ``/cmd_vel``. This fixes the
   worst part of dead-reckoning (heading drift); position still drifts slowly.
3. **Pure dead-reckoning** — integrate yaw *and* XY from ``/cmd_vel`` only.
   Used in stub mode (no SDK / robot unreachable). Drifts fastest.

Position/yaw are zeroed to the first reading so odom starts at the origin
(a goal of ``x:2.0`` means 2 m ahead of where the robot started).

Frames: ``odom`` (parent) -> ``pelvis`` (child).
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# Which SDK odom source is currently driving the estimate.
SRC_NONE = "none"
SRC_SPORT = "sportmodestate"   # full world pose from SDK
SRC_IMU = "imu+cmd_vel"        # IMU yaw + cmd_vel XY integration
SRC_DEADRECKON = "cmd_vel"     # pure cmd_vel integration

# Consider an SDK source "live" only if a message arrived this recently.
_SPORT_TIMEOUT = 0.5  # s
_IMU_TIMEOUT = 0.5    # s
# If no /cmd_vel arrives within this window, assume the robot is stopped (don't
# keep integrating a stale velocity into runaway odom). Only affects the
# dead-reckon / IMU paths; SportModeState reports real velocity directly.
_CMD_TIMEOUT = 0.5    # s


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


try:
    from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    _SDK_AVAILABLE = True
except Exception:
    _SDK_AVAILABLE = False


class OdomBridge(Node):
    def __init__(self):
        super().__init__("odom_bridge")

        # NIC name on the robot subnet (e.g. "enp4s0"), NOT the robot IP — DDS
        # binds to this interface. Empty = auto-detect. Matches robot_node.
        self.declare_parameter("net_iface", "")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "pelvis")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("use_sdk", True)
        # odom_bridge is its own process, so it must init the DDS factory itself
        # (each process owns one ChannelFactory singleton). Leave True unless
        # another node already initialised it in THIS process.
        self.declare_parameter("init_channel_factory", True)
        self.declare_parameter("sportstate_topic", "rt/sportmodestate")
        self.declare_parameter("lowstate_topic", "rt/lowstate")

        self._odom_frame: str = self.get_parameter("odom_frame").value
        self._base_frame: str = self.get_parameter("base_frame").value
        rate: float = float(self.get_parameter("publish_rate_hz").value)
        use_sdk: bool = bool(self.get_parameter("use_sdk").value)

        self._pub = self.create_publisher(Odometry, "/odom", 50)
        self._tf = TransformBroadcaster(self)

        # Integrated / reported state (odom frame, origin-zeroed)
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx_body = 0.0
        self._vy_body = 0.0
        self._wz = 0.0
        self._cmd_stamp = 0.0             # monotonic time of last /cmd_vel

        # Raw SDK readings + freshness timestamps
        self._sport_xyyaw = None          # (x, y, yaw, vx, vy, wz) raw
        self._sport_stamp = 0.0
        self._sport_origin = None         # (x0, y0, yaw0) for zeroing
        self._imu_yaw_raw = None
        self._imu_stamp = 0.0
        self._imu_origin = None
        self._active_src = SRC_NONE       # for change logging

        # Fallback: latest /cmd_vel
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        # SDK subscribers (best-effort)
        self._sport_sub = None
        self._low_sub = None
        if use_sdk and _SDK_AVAILABLE:
            try:
                if bool(self.get_parameter("init_channel_factory").value):
                    iface = self.get_parameter("net_iface").value
                    if iface:
                        ChannelFactory().Init(0, iface)
                    else:
                        ChannelFactory().Init(0)

                sport_topic = self.get_parameter("sportstate_topic").value
                self._sport_sub = ChannelSubscriber(sport_topic, SportModeState_)
                self._sport_sub.Init(self._sport_state_cb, 10)

                low_topic = self.get_parameter("lowstate_topic").value
                self._low_sub = ChannelSubscriber(low_topic, LowState_)
                self._low_sub.Init(self._low_state_cb, 10)

                self.get_logger().info(
                    f"SDK subscribers up: '{sport_topic}' (SportModeState) + "
                    f"'{low_topic}' (LowState/IMU). Source auto-selects on first data."
                )
            except Exception as e:
                self.get_logger().warn(
                    f"SDK subscriber init failed, dead-reckoning only: {e}"
                )
                self._sport_sub = None
                self._low_sub = None
        else:
            self.get_logger().warn(
                "Unitree SDK not available — pure /cmd_vel dead-reckoning for odom."
            )

        period = 1.0 / max(rate, 1.0)
        self._timer = self.create_timer(period, self._tick)
        self._last_tick = time.monotonic()

    # ------------------------------------------------------------------ SDK
    def _sport_state_cb(self, msg: "SportModeState_"):
        try:
            x = float(msg.position[0])
            y = float(msg.position[1])
            yaw = float(msg.imu_state.rpy[2])
            vx = float(msg.velocity[0])
            vy = float(msg.velocity[1])
            wz = float(msg.yaw_speed)
            with self._lock:
                self._sport_xyyaw = (x, y, yaw, vx, vy, wz)
                self._sport_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f"SportModeState parse failed: {e}", throttle_duration_sec=5.0)

    def _low_state_cb(self, msg: "LowState_"):
        try:
            yaw = float(msg.imu_state.rpy[2])
            with self._lock:
                self._imu_yaw_raw = yaw
                self._imu_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f"LowState IMU parse failed: {e}", throttle_duration_sec=5.0)

    # -------------------------------------------------------- /cmd_vel fallback
    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._vx_body = float(msg.linear.x)
            self._vy_body = float(msg.linear.y)
            self._wz = float(msg.angular.z)
            self._cmd_stamp = time.monotonic()

    # ------------------------------------------------------------------ tick
    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        with self._lock:
            sport_live = (
                self._sport_xyyaw is not None
                and (now - self._sport_stamp) < _SPORT_TIMEOUT
            )
            imu_live = (
                self._imu_yaw_raw is not None
                and (now - self._imu_stamp) < _IMU_TIMEOUT
            )
            # Stale /cmd_vel → treat as stopped so we don't integrate runaway.
            cmd_stale = (now - self._cmd_stamp) > _CMD_TIMEOUT
            cvx = 0.0 if cmd_stale else self._vx_body
            cvy = 0.0 if cmd_stale else self._vy_body
            cwz = 0.0 if cmd_stale else self._wz

            if sport_live:
                src = SRC_SPORT
                rx, ry, ryaw, vx, vy, wz = self._sport_xyyaw
                if self._sport_origin is None:
                    self._sport_origin = (rx, ry, ryaw)
                ox, oy, oyaw = self._sport_origin
                # Express relative to origin, rotating into the zeroed frame.
                dx, dy = rx - ox, ry - oy
                c, s = math.cos(-oyaw), math.sin(-oyaw)
                self._x = dx * c - dy * s
                self._y = dx * s + dy * c
                self._yaw = math.atan2(math.sin(ryaw - oyaw), math.cos(ryaw - oyaw))
                self._vx_body, self._vy_body, self._wz = vx, vy, wz

            elif imu_live:
                src = SRC_IMU
                if self._imu_origin is None:
                    self._imu_origin = self._imu_yaw_raw
                yaw = math.atan2(
                    math.sin(self._imu_yaw_raw - self._imu_origin),
                    math.cos(self._imu_yaw_raw - self._imu_origin),
                )
                self._yaw = yaw
                # Integrate XY from cmd_vel using the true (IMU) heading.
                c, s = math.cos(yaw), math.sin(yaw)
                self._x += (cvx * c - cvy * s) * dt
                self._y += (cvx * s + cvy * c) * dt
                self._vx_body, self._vy_body, self._wz = cvx, cvy, cwz

            else:
                src = SRC_DEADRECKON
                c, s = math.cos(self._yaw), math.sin(self._yaw)
                self._x += (cvx * c - cvy * s) * dt
                self._y += (cvx * s + cvy * c) * dt
                self._yaw = math.atan2(
                    math.sin(self._yaw + cwz * dt),
                    math.cos(self._yaw + cwz * dt),
                )
                self._vx_body, self._vy_body, self._wz = cvx, cvy, cwz

            x, y, yaw = self._x, self._y, self._yaw
            vx, vy, wz = self._vx_body, self._vy_body, self._wz

        if src != self._active_src:
            self._active_src = src
            self.get_logger().info(f"Odom source = {src}")

        stamp = self.get_clock().now().to_msg()
        q = _yaw_to_quat(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05
        self._pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = q
        self._tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
