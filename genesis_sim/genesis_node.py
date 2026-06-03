"""
Genesis simulation bridge for G1 human avoidance validation.

Runs Genesis physics + rendering in one thread, publishes ROS2 topics in another.
Validates the full pipeline: SLAM + detection + Nav2 + avoidance logic.

Usage:
    # Terminal 1 — Genesis bridge
    python3 genesis_sim/genesis_node.py

    # Terminal 2 — ROS2 stack
    ros2 launch g1_bringup genesis.launch.py

Requirements:
    pip install genesis-world
    pip install rclpy (via ROS2 Humble)
"""
from __future__ import annotations

import math
import time
import threading
import numpy as np

# ── ROS2 ──────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header, String
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# Avoidance thresholds — must match detection_node params
FRONT_STOP_M      = 1.8
FRONT_SLOW_M      = 3.8
SIDE_STOP_M       = 0.8
FRONT_HALF_WIDTH  = 0.7   # lateral corridor considered "front"
PREDICTION_SECS   = 1.5
PREDICTION_STEPS  = 20

# ── Genesis ───────────────────────────────────────────────────────────────
import genesis as gs

import os
URDF_PATH = os.path.join(
    os.path.dirname(__file__), "..",
    "ros2_ws/src/g1_description/urdf/g1_29dof.urdf"
)

QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Camera intrinsics matching RealSense D435i at 640x480
CAM_W, CAM_H = 640, 480
CAM_FX = 615.0
CAM_FY = 615.0
CAM_CX = 320.0
CAM_CY = 240.0
CAM_FOV_DEG = 2 * math.degrees(math.atan2(CAM_W / 2, CAM_FX))  # ~86 deg


class GenesisNode(Node):
    SIM_DT   = 0.01    # Genesis timestep (100 Hz physics)
    PUB_HZ   = 30      # camera publish rate
    IMU_HZ   = 200     # IMU publish rate
    ODOM_HZ  = 50      # odometry publish rate

    def __init__(self):
        super().__init__("genesis_node")

        self._lock = threading.Lock()

        # Robot state
        self._cmd_vx  = 0.0
        self._cmd_vy  = 0.0
        self._cmd_wz  = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        # Publishers
        self._rgb_pub      = self.create_publisher(Image,       "/camera/color/image_raw",                  QOS_SENSOR)
        self._depth_pub    = self.create_publisher(Image,       "/camera/aligned_depth_to_color/image_raw", QOS_SENSOR)
        self._info_pub     = self.create_publisher(CameraInfo,  "/camera/color/camera_info",                QOS_SENSOR)
        self._imu_pub      = self.create_publisher(Imu,         "/imu_in_torso/data",                       QOS_SENSOR)
        self._odom_pub     = self.create_publisher(Odometry,    "/odom/raw",                                10)
        self._markers_pub  = self.create_publisher(MarkerArray, "/humans/markers",                          10)
        self._human_cmd_pub= self.create_publisher(String,      "/g1/human_cmd",                            10)
        self._tf_br        = TransformBroadcaster(self)

        # cmd_vel subscriber
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        self.get_logger().info("Genesis node ready — waiting for simulation to start.")

    # ── ROS2 callbacks ────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._cmd_vx = float(msg.linear.x)
            self._cmd_vy = float(msg.linear.y)
            self._cmd_wz = float(msg.angular.z)

    def get_cmd_vel(self):
        with self._lock:
            return self._cmd_vx, self._cmd_vy, self._cmd_wz

    # ── Publishers ────────────────────────────────────────────────────────

    def publish_rgb(self, rgb: np.ndarray):
        msg = Image()
        msg.header = self._header("d435_link")
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.step = msg.width * 3
        msg.data = rgb.astype(np.uint8).tobytes()
        self._rgb_pub.publish(msg)

    def publish_depth(self, depth: np.ndarray):
        # depth in metres → convert to uint16 mm for RealSense compatibility
        depth_mm = (depth * 1000.0).astype(np.uint16)
        msg = Image()
        msg.header = self._header("d435_link")
        msg.height, msg.width = depth_mm.shape[:2]
        msg.encoding = "16UC1"
        msg.step = msg.width * 2
        msg.data = depth_mm.tobytes()
        self._depth_pub.publish(msg)

    def publish_camera_info(self):
        msg = CameraInfo()
        msg.header = self._header("d435_link")
        msg.width  = CAM_W
        msg.height = CAM_H
        msg.k = [CAM_FX, 0.0, CAM_CX,
                 0.0, CAM_FY, CAM_CY,
                 0.0, 0.0, 1.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [CAM_FX, 0.0, CAM_CX, 0.0,
                 0.0, CAM_FY, CAM_CY, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(msg)

    def publish_imu(self, lin_acc: np.ndarray, ang_vel: np.ndarray):
        msg = Imu()
        msg.header = self._header("imu_in_torso")
        msg.linear_acceleration.x = float(lin_acc[0])
        msg.linear_acceleration.y = float(lin_acc[1])
        msg.linear_acceleration.z = float(lin_acc[2])
        msg.angular_velocity.x = float(ang_vel[0])
        msg.angular_velocity.y = float(ang_vel[1])
        msg.angular_velocity.z = float(ang_vel[2])
        msg.orientation_covariance[0] = -1.0  # unknown
        self._imu_pub.publish(msg)

    def publish_odom(self, x: float, y: float, yaw: float,
                     vx: float, vy: float, wz: float):
        now = self._header("odom")
        msg = Odometry()
        msg.header = now
        msg.child_frame_id = "pelvis"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = cy
        msg.pose.pose.orientation.z = sy
        msg.twist.twist.linear.x  = vx
        msg.twist.twist.linear.y  = vy
        msg.twist.twist.angular.z = wz
        self._odom_pub.publish(msg)

        # TF: odom → pelvis
        tf = TransformStamped()
        tf.header = now
        tf.child_frame_id = "pelvis"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.8  # approx pelvis height
        tf.transform.rotation.w = cy
        tf.transform.rotation.z = sy
        self._tf_br.sendTransform(tf)

    def publish_ground_truth_humans(
        self,
        humans: list,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ):
        """
        Bypass YOLO — publish known human positions directly as /humans/markers
        and compute /g1/human_cmd from distance thresholds.
        Same message format as detection_node so human_obstacle_node and
        robot_node work unchanged.
        """
        marker_array = MarkerArray()
        worst_cmd = "NORMAL_OPERATION"

        for i, human in enumerate(humans):
            hx, hy = float(human.pos[0]), float(human.pos[1])
            vx, vy = float(human.vel[0]), float(human.vel[1])

            # ── Current position marker (cylinder) ────────────────────
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "humans"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = hx
            m.pose.position.y = hy
            m.pose.position.z = 0.9
            m.pose.orientation.w = 1.0
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 1.8
            m.color.r = 1.0
            m.color.a = 0.8
            m.lifetime.sec = 1
            marker_array.markers.append(m)

            # ── Predicted trajectory (linear projection) ───────────────
            speed = math.hypot(vx, vy)
            if speed > 0.05:
                pred_marker = Marker()
                pred_marker.header.frame_id = "map"
                pred_marker.header.stamp = m.header.stamp
                pred_marker.ns = "human_predictions"
                pred_marker.id = i + 1000
                pred_marker.type = Marker.LINE_STRIP
                pred_marker.action = Marker.ADD
                pred_marker.scale.x = 0.05
                pred_marker.color.g = 1.0
                pred_marker.color.a = 0.8
                pred_marker.lifetime.sec = 1

                dt = PREDICTION_SECS / PREDICTION_STEPS
                px, py = hx, hy
                for _ in range(PREDICTION_STEPS):
                    px += vx * dt
                    py += vy * dt
                    pt = Point()
                    pt.x, pt.y, pt.z = px, py, 0.0
                    pred_marker.points.append(pt)
                marker_array.markers.append(pred_marker)

            # ── Compute avoidance command ──────────────────────────────
            # Transform human pos into robot frame
            dx = hx - robot_x
            dy = hy - robot_y
            # Rotate to robot frame
            rel_fwd  =  dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
            rel_side = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)
            dist = math.hypot(rel_fwd, rel_side)

            in_front = rel_fwd >= 0 and abs(rel_side) <= FRONT_HALF_WIDTH

            if in_front:
                if dist < FRONT_STOP_M:
                    worst_cmd = "STOP"
                elif dist < FRONT_SLOW_M and worst_cmd != "STOP":
                    worst_cmd = "SLOW_DOWN"
            else:
                if dist < SIDE_STOP_M:
                    worst_cmd = "STOP"

        self._markers_pub.publish(marker_array)

        cmd_msg = String()
        cmd_msg.data = worst_cmd
        self._human_cmd_pub.publish(cmd_msg)

    def _header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h


class HumanActor:
    """A scripted human that walks a patrol path in the Genesis scene."""

    def __init__(self, scene, waypoints: list, speed: float = 0.8):
        self._waypoints = waypoints
        self._speed = speed
        self._idx = 0
        self.pos = np.array(waypoints[0], dtype=float)
        self.vel = np.zeros(3, dtype=float)   # exposed for ground truth publishing

        self._entity = scene.add_entity(
            gs.morphs.Cylinder(radius=0.2, height=1.8),
            surface=gs.surfaces.Default(color=(0.8, 0.3, 0.3, 1.0)),
        )

    def step(self, dt: float):
        target = np.array(self._waypoints[self._idx], dtype=float)
        direction = target - self.pos
        dist = np.linalg.norm(direction)

        if dist < 0.1:
            self._idx = (self._idx + 1) % len(self._waypoints)
            self.vel = np.zeros(3)
        else:
            self.vel = (direction / dist) * self._speed
            self.pos += self.vel * dt

        self._entity.set_pos(self.pos)


def build_scene(node: GenesisNode):
    """Build and run the Genesis scene."""

    gs.init(backend=gs.cpu, logging_level="warning")

    scene = gs.Scene(
        show_viewer=False,
        sim_options=gs.options.SimOptions(dt=node.SIM_DT),
    )

    # Ground plane
    scene.add_entity(gs.morphs.Plane())

    # Walls (matching test_room.world — 10x10m room)
    for pos, size in [
        ((0,  5, 1), (10, 0.2, 2)),
        ((0, -5, 1), (10, 0.2, 2)),
        (( 5, 0, 1), (0.2, 10, 2)),
        ((-5, 0, 1), (0.2, 10, 2)),
    ]:
        scene.add_entity(
            gs.morphs.Box(size=size),
            pos=pos,
        )

    # G1 robot — floating base (gravity disabled via zero gravity + velocity control)
    robot = scene.add_entity(
        gs.morphs.URDF(
            file=os.path.abspath(URDF_PATH),
            fixed=False,
            merge_fixed_links=True,
        ),
        pos=(0, 0, 0.8),
    )

    # RGB-D camera at d435_link position relative to torso
    # d435_joint origin: xyz="0.0576235 0.01753 0.41987" rpy="0 0.831 0"
    camera = scene.add_camera(
        res=(CAM_W, CAM_H),
        pos=(0.06, 0.02, 1.22),   # approx world position when robot at z=0.8
        lookat=(1.0, 0.0, 0.8),
        fov=CAM_FOV_DEG,
        GUI=False,
    )

    # Human actors — patrol paths across the room
    humans = [
        HumanActor(scene, waypoints=[(2, -3, 0), (2, 3, 0)], speed=0.7),
        HumanActor(scene, waypoints=[(-2, 3, 0), (3, -1, 0)], speed=0.5),
    ]

    scene.build()

    node.get_logger().info("Genesis scene built. Starting simulation loop.")

    # ── Timing ────────────────────────────────────────────────────────────
    cam_interval  = 1.0 / node.PUB_HZ
    imu_interval  = 1.0 / node.IMU_HZ
    odom_interval = 1.0 / node.ODOM_HZ

    last_cam  = 0.0
    last_imu  = 0.0
    last_odom = 0.0

    sim_time = 0.0
    robot_yaw = 0.0
    robot_x, robot_y = 0.0, 0.0

    prev_lin_vel = np.zeros(3)

    # ── Main simulation loop ──────────────────────────────────────────────
    while rclpy.ok():
        t_start = time.perf_counter()

        # Get velocity command from ROS2
        vx, vy, wz = node.get_cmd_vel()

        # Update robot pose (kinematic floating base — no fall)
        dt = node.SIM_DT
        robot_yaw += wz * dt
        robot_x   += (vx * math.cos(robot_yaw) - vy * math.sin(robot_yaw)) * dt
        robot_y   += (vx * math.sin(robot_yaw) + vy * math.cos(robot_yaw)) * dt

        cy, sy = math.cos(robot_yaw), math.sin(robot_yaw)

        # Move robot in Genesis
        robot.set_pos((robot_x, robot_y, 0.8))
        robot.set_quat((0.0, 0.0, sy * 0.7071, cy * 0.7071))  # approx yaw quat

        # Move camera to follow robot
        cam_x = robot_x + 0.06 * cy
        cam_y = robot_y + 0.06 * sy
        camera.set_pose(
            pos=(cam_x, cam_y, 0.8 + 0.42),
            lookat=(cam_x + cy, cam_y + sy, 0.8 + 0.42),
        )

        # Step humans
        for h in humans:
            h.step(dt)

        # Step physics
        scene.step()
        sim_time += dt

        # ── Camera publish ─────────────────────────────────────────────
        if sim_time - last_cam >= cam_interval:
            last_cam = sim_time
            rgb, depth = camera.render(rgb=True, depth=True)
            node.publish_rgb(rgb)
            node.publish_depth(depth)
            node.publish_camera_info()

        # ── IMU publish ────────────────────────────────────────────────
        if sim_time - last_imu >= imu_interval:
            last_imu = sim_time
            lin_acc = np.array([0.0, 0.0, 9.81])  # static gravity
            ang_vel = np.array([0.0, 0.0, wz])
            node.publish_imu(lin_acc, ang_vel)

        # ── Odometry publish ───────────────────────────────────────────
        if sim_time - last_odom >= odom_interval:
            last_odom = sim_time
            node.publish_odom(robot_x, robot_y, robot_yaw, vx, vy, wz)

        # ── Ground truth human positions (bypasses YOLO) ───────────────
        # Publishes /humans/markers and /g1/human_cmd at camera rate
        if sim_time - last_cam >= cam_interval:
            node.publish_ground_truth_humans(
                humans, robot_x, robot_y, robot_yaw
            )

        # ── Spin ROS2 once ─────────────────────────────────────────────
        rclpy.spin_once(node, timeout_sec=0.0)

        # ── Rate limiting ──────────────────────────────────────────────
        elapsed = time.perf_counter() - t_start
        sleep_t = node.SIM_DT - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


def main():
    rclpy.init()
    node = GenesisNode()

    try:
        build_scene(node)
    except KeyboardInterrupt:
        node.get_logger().info("Genesis simulation stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
