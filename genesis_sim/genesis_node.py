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
MJCF_PATH = os.path.join(os.path.dirname(__file__), "g1_29dof.xml")
URDF_PATH = os.path.join(
    os.path.dirname(__file__), "..",
    "ros2_ws/src/g1_description/urdf/g1_29dof.urdf"
)

QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Low resolution for CPU rendering speed — RTAB-Map works fine at 160x120
CAM_W, CAM_H = 160, 120
CAM_FX = 154.0   # scaled from 615 * (160/640)
CAM_FY = 154.0
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

        # Human avoidance state (sim-equivalent of robot_node's override)
        self._human_cmd = "NORMAL_OPERATION"

        # Publishers
        # Camera topics use RELIABLE QoS (depth=10) so RTAB-Map can sync timestamps
        self._rgb_pub      = self.create_publisher(Image,       "/camera/color/image_raw",                  10)
        self._depth_pub    = self.create_publisher(Image,       "/camera/aligned_depth_to_color/image_raw", 10)
        self._info_pub     = self.create_publisher(CameraInfo,  "/camera/color/camera_info",                10)
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

    def publish_camera_frame(self, rgb: np.ndarray, depth: np.ndarray):
        """Publish RGB, depth and camera_info with identical timestamps for RTAB-Map sync."""
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = "d435_link"

        # RGB
        rgb_msg = Image()
        rgb_msg.header = header
        rgb_msg.height, rgb_msg.width = rgb.shape[:2]
        rgb_msg.encoding = "rgb8"
        rgb_msg.step = rgb_msg.width * 3
        rgb_msg.data = rgb.astype(np.uint8).tobytes()
        self._rgb_pub.publish(rgb_msg)

        # Depth (mm as uint16)
        depth_mm = (depth * 1000.0).astype(np.uint16)
        depth_msg = Image()
        depth_msg.header = header
        depth_msg.height, depth_msg.width = depth_mm.shape[:2]
        depth_msg.encoding = "16UC1"
        depth_msg.step = depth_msg.width * 2
        depth_msg.data = depth_mm.tobytes()
        self._depth_pub.publish(depth_msg)

        # Camera info — same stamp
        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.width  = rgb_msg.width
        info_msg.height = rgb_msg.height
        info_msg.k = [CAM_FX, 0.0, CAM_CX, 0.0, CAM_FY, CAM_CY, 0.0, 0.0, 1.0]
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [CAM_FX, 0.0, CAM_CX, 0.0, 0.0, CAM_FY, CAM_CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info_msg)

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

        # Store for the velocity override applied in the main loop
        with self._lock:
            self._human_cmd = worst_cmd

        cmd_msg = String()
        cmd_msg.data = worst_cmd
        self._human_cmd_pub.publish(cmd_msg)

    def get_motion_scale(self):
        """
        Velocity scale based on human proximity — the sim-equivalent of
        robot_node's safety override on real hardware.
          STOP             -> 0.0 (block all motion)
          SLOW_DOWN        -> 0.4
          NORMAL_OPERATION -> 1.0
        """
        with self._lock:
            cmd = self._human_cmd
        if cmd == "STOP":
            return 0.0
        if cmd == "SLOW_DOWN":
            return 0.4
        return 1.0

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
    # Use same small resolution in both modes — CPU rendering is the bottleneck
    cam_w  = CAM_W
    cam_h  = CAM_H
    cam_hz = node.PUB_HZ  # target 10Hz — actual rate depends on CPU speed

    gs.init(backend=backend, logging_level="warning")

    scene = gs.Scene(
        show_viewer=use_viewer,
        sim_options=gs.options.SimOptions(
            dt=node.SIM_DT,
            gravity=(0.0, 0.0, 0.0),  # disable gravity — kinematic robot + static walls
        ),
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

    # Robot model selection:
    # - Kinematic mode → URDF (set_pos works reliably)
    # - RL policy mode → MJCF (correct DOF structure for policy control)
    if use_rl:
        robot = scene.add_entity(
            gs.morphs.MJCF(
                file=os.path.abspath(MJCF_PATH),
                pos=(0, 0, 0),
            ),
        )
    else:
        robot = scene.add_entity(
            gs.morphs.URDF(
                file=os.path.abspath(URDF_PATH),
                fixed=False,
                merge_fixed_links=False,
                pos=(0, 0, 0.85),
            ),
        )

    # D435 mount from URDF: xyz="0.0576235 0.01753 0.41987" on torso_link
    # torso_link world z = pelvis(0.793) + waist_roll(0.035) + torso(0.019) = 0.847
    # d435 world z = 0.847 + 0.41987 = 1.267
    # pitch = 0.831 rad (~47.6°) downward — camera looks forward and down
    D435_WORLD_X  =  0.0576
    D435_WORLD_Y  =  0.0175
    D435_WORLD_Z  =  1.267
    D435_PITCH    =  0.831   # rad downward

    # Compute lookat point from pitch: camera looks along pitched-forward direction
    import math as _math
    _look_dist = 2.0
    _look_x = D435_WORLD_X + _look_dist * _math.cos(D435_PITCH)
    _look_z = D435_WORLD_Z - _look_dist * _math.sin(D435_PITCH)

    camera = scene.add_camera(
        res=(cam_w, cam_h),
        pos=(D435_WORLD_X, D435_WORLD_Y, D435_WORLD_Z),
        lookat=(_look_x, D435_WORLD_Y, _look_z),
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
    last_human  = 0.0
    last_policy = 0.0

    sim_time  = 0.0
    robot_yaw = 0.0
    robot_x, robot_y = 0.0, 0.0

    # ── Main simulation loop ──────────────────────────────────────────────
    while rclpy.ok():
        t_start = time.perf_counter()
        dt = node.SIM_DT
        vx, vy, wz = node.get_cmd_vel()

        # Human avoidance override: scale Nav2's velocity by human proximity.
        # On hardware this happens in robot_node; in sim genesis_node is the
        # actuator, so the override is applied here.
        scale = node.get_motion_scale()
        vx *= scale
        vy *= scale
        wz *= scale

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

        else:
            # ── KINEMATIC MODE (no policy) ─────────────────────────────
            robot_yaw += wz * dt
            robot_x   += (vx * math.cos(robot_yaw) - vy * math.sin(robot_yaw)) * dt
            robot_y   += (vx * math.sin(robot_yaw) + vy * math.cos(robot_yaw)) * dt
            qw = math.cos(robot_yaw / 2)
            qz = math.sin(robot_yaw / 2)
            robot.set_pos((robot_x, robot_y, 0.85))
            robot.set_quat((qw, 0.0, 0.0, qz))

        # cy/sy always available after both branches
        cy, sy = math.cos(robot_yaw), math.sin(robot_yaw)

        # Move camera to follow robot — exact D435 offset from torso
        # d435_joint xyz="0.0576235 0.01753 0.41987" on torso_link, pitch=0.831 rad
        cam_x = robot_x + D435_WORLD_X * cy - D435_WORLD_Y * sy
        cam_y = robot_y + D435_WORLD_X * sy + D435_WORLD_Y * cy
        cam_z = D435_WORLD_Z  # stays constant in kinematic mode
        look_x = cam_x + _look_dist * math.cos(D435_PITCH) * cy
        look_y = cam_y + _look_dist * math.cos(D435_PITCH) * sy
        look_z = cam_z - _look_dist * math.sin(D435_PITCH)
        camera.set_pose(pos=(cam_x, cam_y, cam_z), lookat=(look_x, look_y, look_z))

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
            node.publish_camera_frame(rgb, depth)

        # ── IMU publish ────────────────────────────────────────────────
        if sim_time - last_imu >= imu_interval:
            last_imu = sim_time
            lin_acc = np.array([0.0, 0.0, 9.81])  # static gravity
            ang_vel = np.array([0.0, 0.0, wz])
            node.publish_imu(lin_acc, ang_vel)

        # ── Odometry + TF — publish every step for smooth RViz ────────
        node.publish_odom(robot_x, robot_y, robot_yaw, vx, vy, wz)

        # ── Ground truth human positions (bypasses YOLO) ───────────────
        if sim_time - last_human >= cam_interval:
            last_human = sim_time
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
