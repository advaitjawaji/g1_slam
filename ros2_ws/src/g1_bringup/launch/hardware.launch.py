"""
Full hardware bringup for Unitree G1 autonomous human avoidance demo.

Components launched:
  1. Camera (RealSense D435i or ZED 2i)
  2. Robot state publisher (G1 URDF)
  3. RTAB-Map SLAM
  4. Human detection + obstacle injection
  5. Nav2 navigation stack
  6. G1 locomotion node (Unitree SDK)
  7. RViz (optional)

Usage:
    # RealSense (default)
    ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161

    # ZED 2i
    ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161 camera:=zed

Arguments:
    robot_ip    — G1 IP address (default: 192.168.123.161)
    camera      — Camera type: 'realsense' or 'zed' (default: realsense)
    model_path  — YOLO model file (default: yolo11n.pt, use yolo26n.engine on Jetson)
    rviz        — Launch RViz visualizer (default: true)
    slam_mode   — 'mapping' builds new map, 'localization' uses existing (default: mapping)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip   = LaunchConfiguration("robot_ip",   default="192.168.123.161")
    net_iface  = LaunchConfiguration("net_iface",  default="")
    camera     = LaunchConfiguration("camera",     default="realsense")
    model_path = LaunchConfiguration("model_path", default="yolo11n.pt")
    rviz       = LaunchConfiguration("rviz",       default="true")
    slam_mode  = LaunchConfiguration("slam_mode",  default="mapping")

    use_zed        = PythonExpression(["'", camera, "' == 'zed'"])
    use_realsense  = PythonExpression(["'", camera, "' != 'zed'"])

    pkg_desc    = get_package_share_directory("g1_description")
    pkg_bringup = get_package_share_directory("g1_bringup")

    urdf_path      = os.path.join(pkg_desc, "urdf", "g1_29dof.urdf")
    nav2_params    = os.path.join(pkg_bringup, "config", "nav2_params_hw.yaml")
    rviz_config    = os.path.join(pkg_bringup, "config", "demo.rviz")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ── 1. Robot description ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # ── 2a. RealSense D435i ───────────────────────────────────────────────
    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        parameters=[{
            "enable_color":               True,
            "enable_depth":               True,
            "enable_gyro":                True,
            "enable_accel":               True,
            "unite_imu_method":           "linear_interpolation",
            "align_depth.enable":         True,
            "depth_module.depth_profile": "640x480x30",
            "rgb_camera.color_profile":   "640x480x30",
        }],
        condition=IfCondition(use_realsense),
    )

    # ── 2b. ZED 2i ───────────────────────────────────────────────────────
    # ZED ROS2 wrapper publishes to /zed/zed_node/* — we remap to our
    # standard /camera/* topics so SLAM and detection work unchanged.
    # Install: https://github.com/stereolabs/zed-ros2-wrapper
    zed = Node(
        package="zed_wrapper",
        executable="zed_wrapper",
        name="zed_node",
        namespace="zed",
        parameters=[{
            "general.camera_model":        "zed2i",   # this unit is a ZED 2i
            "general.grab_resolution":     "HD720",
            "general.grab_frame_rate":     30,
            "depth.depth_mode":            "ULTRA",
            "sensors.publish_imu_tf":      False,
            # ── VIO odometry: the odom source for the whole nav stack ──────
            # The wrapper hard-codes its base to <camera>_camera_link (there is
            # NO base_frame param), so it publishes `odom -> zed_camera_link`.
            # We attach `pelvis` under it via the zed_camera_link->pelvis static
            # TF below. publish_map_tf is OFF: we navigate in the `odom` frame
            # (nav2_params_hw global_frame=odom).
            # NOTE: this bare node does NOT load common_stereo.yaml — every
            # non-default setting must be listed here.
            "pos_tracking.pos_tracking_enabled":  True,
            "pos_tracking.publish_tf":            True,
            "pos_tracking.publish_map_tf":        False,
        }],
        remappings=[
            ("zed_node/rgb/image_rect_color",        "/camera/color/image_raw"),
            ("zed_node/rgb/camera_info",             "/camera/color/camera_info"),
            ("zed_node/depth/depth_registered",      "/camera/aligned_depth_to_color/image_raw"),
            ("zed_node/point_cloud/cloud_registered", "/camera/depth/points"),
            ("zed_node/imu/data",                    "/imu_in_torso/data"),
            ("zed_node/odom",                        "/odom"),
        ],
        condition=IfCondition(use_zed),
    )

    # Physical ZED mount: head, 5 cm in front of and 40 cm above the pelvis,
    # pitched ~17.5° (0.3054 rad) DOWN. The wrapper publishes odom->zed_camera_link,
    # so zed_camera_link is the PARENT and pelvis hangs under it (pelvis is the
    # URDF root, free to take a parent). This also replaces the old
    # zed_left_camera_frame->d435_link alias, which double-parented d435_link.
    #
    # These are the INVERSE of the intuitive pelvis->camera mount
    # (cam at +0.05,0,+0.40 level, pitch +0.3054), because we publish camera->pelvis:
    #   R = Ry(-0.3054);  t = -R * (0.05,0,0.40) = (0.0726, 0, -0.3965)
    # Args order: x y z yaw pitch roll parent child.
    # TODO: refine pitch once measured (you estimated 15-20°); if you change it,
    # recompute t (or split into camera->level->pelvis statics).
    zed_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_to_pelvis_tf",
        arguments=["0.0726", "0.0", "-0.3965", "0", "-0.3054", "0",
                   "zed_camera_link", "pelvis"],
        condition=IfCondition(use_zed),
    )

    # ── 3. RTAB-Map SLAM ─────────────────────────────────────────────────
    slam_launch_file = PythonExpression([
        "'slam.launch.py' if '", slam_mode, "' == 'mapping' else 'localization.launch.py'"
    ])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("g1_slam"), "launch", "slam.launch.py"])
        ]),
        launch_arguments={
            "use_sim":       "false",
            "launch_camera": "false",   # RealSense already launched above
            "viz":           "false",   # use RViz below instead
        }.items(),
    )

    # ── 4. Human detection (delay to let SLAM initialise) ────────────────
    detection = TimerAction(
        period=8.0,
        actions=[Node(
            package="g1_detection",
            executable="detection_node",
            name="detection_node",
            output="screen",
            parameters=[{
                "model_path":   model_path,
                "conf":         0.25,
                "imgsz":        384,
                # ZED path uses the camera's own frame (d435_link alias removed);
                # RealSense path keeps the URDF mount frame.
                "camera_frame": PythonExpression(
                    ["'zed_left_camera_frame' if '", camera, "' == 'zed' else 'd435_link'"]),
                # Nav runs in the `odom` frame (no global map); publish detections
                # there so they match the costmaps. Falls back to camera frame if
                # odom isn't up yet.
                "map_frame":    "odom",
            }],
        )],
    )

    # ── 5. Human obstacle injection ──────────────────────────────────────
    human_obstacle = TimerAction(
        period=8.0,
        actions=[Node(
            package="g1_detection",
            executable="human_obstacle_node",
            name="human_obstacle_node",
            output="screen",
            parameters=[{
                "map_frame":                "odom",   # odom-frame nav (no global map)
                "prediction_horizon_steps": 10,
            }],
        )],
    )

    # ── 6. Nav2 navigation stack ─────────────────────────────────────────
    nav2 = TimerAction(
        period=12.0,   # wait for SLAM to have a map
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
            ]),
            launch_arguments={
                "use_sim_time": "false",
                "params_file":  nav2_params,
            }.items(),
        )],
    )

    # ── 7. G1 locomotion node ────────────────────────────────────────────
    robot_node = TimerAction(
        period=5.0,   # wait for SDK connection
        actions=[Node(
            package="g1_robot",
            executable="robot_node",
            name="robot_node",
            output="screen",
            parameters=[{
                "robot_ip":   robot_ip,
                "robot_port": 8080,
                "net_iface":  net_iface,
            }],
        )],
    )

    # ── 8. RViz ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip",   default_value="192.168.123.161",
                              description="G1 robot IP address"),
        DeclareLaunchArgument("net_iface",  default_value="",
                              description="Host NIC on robot subnet for DDS (e.g. enp4s0); empty=auto"),
        DeclareLaunchArgument("camera",     default_value="realsense",
                              description="Camera type: 'realsense' or 'zed'"),
        DeclareLaunchArgument("model_path", default_value="yolo11n.pt",
                              description="YOLO model path (use yolo26n.engine on Jetson)"),
        DeclareLaunchArgument("rviz",       default_value="true",
                              description="Launch RViz"),
        DeclareLaunchArgument("slam_mode",  default_value="mapping",
                              description="'mapping' or 'localization'"),

        robot_state_publisher,
        realsense,
        zed,
        zed_tf,
        slam,
        detection,
        human_obstacle,
        nav2,
        robot_node,
        rviz_node,
    ])
