"""
Genesis simulation bridge for G1 human avoidance validation.

Runs Genesis physics + rendering in one thread, publishes ROS2 topics in another.
Validates the full pipeline: SLAM + detection + Nav2 + avoidance logic.

Usage:
    # Without RL policy (kinematic floating):
    python3 genesis_sim/genesis_node.py

    # With RL policy (realistic walking):
    python3 genesis_sim/genesis_node.py --policy /path/to/g1_policy.pt

    Download policy checkpoint from:
    https://github.com/unitreerobotics/unitree_rl_gym  (g1/policy.pt)

    # Terminal 2 — ROS2 stack
    ros2 launch g1_bringup genesis.launch.py

Requirements:
    pip install genesis-world torch
    pip install rclpy (via ROS2 Humble)
"""
from __future__ import annotations

import argparse
import math
import os
import sys
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

# ── RL Policy ─────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy import (
    G1LocomotionPolicy, POLICY_JOINT_NAMES, DEFAULT_JOINT_POS,
    NUM_POLICY_JOINTS,
)

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
    SIM_DT   = 0.02    # Genesis timestep (50 Hz physics — easier to keep up with)
    PUB_HZ   = 10      # camera publish rate (lower = less stutter)
    IMU_HZ   = 50      # IMU publish rate
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
            gs.morphs.Cylinder(radius=0.2, height=1.8, pos=waypoints[0]),
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


def build_scene(node: GenesisNode, policy_path: str | None = None, use_viewer: bool = False):
    """Build and run the Genesis scene."""

    # ── RL Policy ─────────────────────────────────────────────────────────
    policy = G1LocomotionPolicy(checkpoint_path=policy_path)
    use_rl = policy_path is not None
    mode = "RL walking" if use_rl else "kinematic floating"
    node.get_logger().info(f"Locomotion mode: {mode}")

    # Viewer mode: use GPU but reduce camera res + rate to avoid double-render lag
    backend = gs.cuda if use_viewer else gs.cpu
    cam_w   = 320 if use_viewer else CAM_W
    cam_h   = 240 if use_viewer else CAM_H
    cam_hz  = 5   if use_viewer else node.PUB_HZ

    gs.init(backend=backend, logging_level="warning")

    scene = gs.Scene(
        show_viewer=use_viewer,
        sim_options=gs.options.SimOptions(dt=node.SIM_DT),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(5, -5, 4),
            camera_lookat=(0, 0, 1),
            camera_fov=50,
        ) if use_viewer else None,
    )

    # Ground plane
    scene.add_entity(gs.morphs.Plane())

    # Walls (matching test_room.world — 10x10m room)
    for wpos, size in [
        ((0,  5, 1), (10, 0.2, 2)),
        ((0, -5, 1), (10, 0.2, 2)),
        (( 5, 0, 1), (0.2, 10, 2)),
        ((-5, 0, 1), (0.2, 10, 2)),
    ]:
        scene.add_entity(gs.morphs.Box(size=size, pos=wpos))

    # G1 robot
    robot = scene.add_entity(
        gs.morphs.URDF(
            file=os.path.abspath(URDF_PATH),
            fixed=False,
            merge_fixed_links=False,
            pos=(0, 0, 0.85),
        ),
    )

    # Camera at d435_link (xyz="0.0576235 0.01753 0.41987" from torso_link)
    camera = scene.add_camera(
        res=(cam_w, cam_h),
        pos=(0.06, 0.02, 1.22),
        lookat=(1.0, 0.0, 0.8),
        fov=CAM_FOV_DEG,
        GUI=False,
    )

    # Human actors
    humans = [
        HumanActor(scene, waypoints=[(2, -3, 0), (2, 3, 0)], speed=0.7),
        HumanActor(scene, waypoints=[(-2, 3, 0), (3, -1, 0)], speed=0.5),
    ]

    scene.build()

    # Get joint indices for policy-controlled joints
    if use_rl:
        policy_joint_indices = []
        for name in POLICY_JOINT_NAMES:
            try:
                idx = robot.get_joint(name).dof_idx_local
                policy_joint_indices.append(idx)
            except Exception:
                node.get_logger().warn(f"Joint not found: {name}")
        policy_joint_indices = np.array(policy_joint_indices, dtype=int)

        # Initialise joints to default standing pose
        init_pos = np.zeros(robot.n_dofs)
        init_pos[policy_joint_indices] = DEFAULT_JOINT_POS
        robot.set_dofs_position(init_pos)

    node.get_logger().info(f"Genesis scene built. Starting {mode} loop.")

    # ── Timing ────────────────────────────────────────────────────────────
    cam_interval   = 1.0 / cam_hz
    imu_interval   = 1.0 / node.IMU_HZ
    odom_interval  = 1.0 / node.ODOM_HZ
    policy_interval = policy._policy_dt if use_rl else 9999

    last_cam    = 0.0
    last_imu    = 0.0
    last_odom   = 0.0
    last_policy = 0.0

    sim_time  = 0.0
    robot_yaw = 0.0
    robot_x, robot_y = 0.0, 0.0

    # ── Main simulation loop ──────────────────────────────────────────────
    while rclpy.ok():
        t_start = time.perf_counter()
        dt = node.SIM_DT
        vx, vy, wz = node.get_cmd_vel()

        if use_rl:
            # ── RL POLICY MODE ─────────────────────────────────────────
            # Run policy at 50Hz, physics at 100Hz
            if sim_time - last_policy >= policy_interval:
                last_policy = sim_time

                # Read joint state from Genesis
                all_pos = robot.get_dofs_position().cpu().numpy()
                all_vel = robot.get_dofs_velocity().cpu().numpy()
                joint_pos = all_pos[policy_joint_indices]
                joint_vel = all_vel[policy_joint_indices]

                # Read base state
                base_quat = robot.get_quat().cpu().numpy()   # (w, x, y, z)
                base_ang_vel = robot.get_ang_vel().cpu().numpy()

                # Gravity vector in base frame
                gravity_world = np.array([0, 0, -1], dtype=np.float32)
                w, x, y, z = base_quat
                # Rotate gravity to base frame using quaternion conjugate
                gravity_base = _rotate_vec_by_quat_inv(gravity_world, np.array([w, x, y, z]))

                # Command from Nav2
                command = np.array([vx, vy, wz], dtype=np.float32)

                # Run policy
                target_pos = policy.step(
                    ang_vel_base=base_ang_vel.astype(np.float32),
                    gravity_vec_base=gravity_base,
                    command=command,
                    joint_pos=joint_pos.astype(np.float32),
                    joint_vel=joint_vel.astype(np.float32),
                )

                # Apply as position targets via PD control
                torques = policy.compute_torques(target_pos, joint_pos, joint_vel)
                full_torques = np.zeros(robot.n_dofs)
                full_torques[policy_joint_indices] = torques
                robot.set_dofs_force(full_torques)

            # Read back actual robot base position for odom/camera
            base_pos = robot.get_pos().cpu().numpy()
            base_quat = robot.get_quat().cpu().numpy()
            robot_x, robot_y = float(base_pos[0]), float(base_pos[1])
            robot_yaw = _quat_to_yaw(base_quat)
            cy, sy = math.cos(robot_yaw), math.sin(robot_yaw)

        else:
            # ── KINEMATIC MODE (no policy) ─────────────────────────────
            robot_yaw += wz * dt
            robot_x   += (vx * math.cos(robot_yaw) - vy * math.sin(robot_yaw)) * dt
            robot_y   += (vx * math.sin(robot_yaw) + vy * math.cos(robot_yaw)) * dt
            # Genesis quaternion format: (w, x, y, z) — yaw rotation around Z axis
            qw = math.cos(robot_yaw / 2)
            qz = math.sin(robot_yaw / 2)
            robot.set_pos((robot_x, robot_y, 0.85))
            robot.set_quat((qw, 0.0, 0.0, qz))

        # Move camera to follow robot
        cam_x = robot_x + 0.06 * cy
        cam_y = robot_y + 0.06 * sy
        camera.set_pose(
            pos=(cam_x, cam_y, 0.85 + 0.42),
            lookat=(cam_x + cy, cam_y + sy, 0.85 + 0.42),
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
            rgb, depth, _, _ = camera.render(rgb=True, depth=True)
            node.publish_rgb(rgb)
            node.publish_depth(depth)
            node.publish_camera_info()

        # ── IMU publish ────────────────────────────────────────────────
        if sim_time - last_imu >= imu_interval:
            last_imu = sim_time
            lin_acc = np.array([0.0, 0.0, 9.81])  # static gravity
            ang_vel = np.array([0.0, 0.0, wz])
            node.publish_imu(lin_acc, ang_vel)

        # ── Odometry + TF — publish every step for smooth RViz ────────
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


def _quat_to_yaw(quat: np.ndarray) -> float:
    """Extract yaw from quaternion (w, x, y, z)."""
    w, x, y, z = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rotate_vec_by_quat_inv(vec: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Rotate vector by inverse of quaternion (w, x, y, z) — world to base frame."""
    w, x, y, z = quat
    # Conjugate (inverse for unit quat)
    qc = np.array([w, -x, -y, -z])
    return _quat_rotate(vec, qc)


def _quat_rotate(vec: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Rotate 3D vector by quaternion (w, x, y, z)."""
    w, x, y, z = quat
    R = np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)
    return (R @ vec).astype(np.float32)


def main():
    parser = argparse.ArgumentParser(description="Genesis G1 simulation bridge")
    parser.add_argument(
        "--policy", type=str, default=None,
        help="Path to RL policy checkpoint (.pt). "
             "Download from https://github.com/unitreerobotics/unitree_rl_gym"
    )
    parser.add_argument(
        "--viewer", action="store_true",
        help="Enable Genesis GPU viewer window (requires NVIDIA GPU)"
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GenesisNode()

    if args.policy:
        node.get_logger().info(f"Loading RL policy: {args.policy}")
    else:
        node.get_logger().info("No policy provided — using kinematic floating mode.")

    try:
        build_scene(node, policy_path=args.policy, use_viewer=args.viewer)
    except KeyboardInterrupt:
        node.get_logger().info("Genesis simulation stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
