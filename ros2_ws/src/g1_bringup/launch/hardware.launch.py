"""
Full hardware bringup for Unitree G1 autonomous human avoidance demo.

Components launched:
  1. RealSense D435i camera
  2. Robot state publisher (G1 URDF)
  3. RTAB-Map SLAM
  4. Human detection + obstacle injection
  5. Nav2 navigation stack
  6. G1 locomotion node (Unitree SDK)
  7. RViz (optional)

Usage:
    ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161

Arguments:
    robot_ip    — G1 IP address (default: 192.168.123.161)
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip   = LaunchConfiguration("robot_ip",   default="192.168.123.161")
    model_path = LaunchConfiguration("model_path", default="yolo11n.pt")
    rviz       = LaunchConfiguration("rviz",       default="true")
    slam_mode  = LaunchConfiguration("slam_mode",  default="mapping")

    pkg_desc    = get_package_share_directory("g1_description")
    pkg_bringup = get_package_share_directory("g1_bringup")

    urdf_path      = os.path.join(pkg_desc, "urdf", "g1_29dof.urdf")
    nav2_params    = os.path.join(pkg_bringup, "config", "nav2_params.yaml")
    rviz_config    = os.path.join(pkg_bringup, "config", "demo.rviz")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ── 1. Robot description ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # ── 2. RealSense D435i ───────────────────────────────────────────────
    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        parameters=[{
            "enable_color":              True,
            "enable_depth":              True,
            "enable_gyro":               True,
            "enable_accel":              True,
            "unite_imu_method":          "linear_interpolation",
            "align_depth.enable":        True,
            "depth_module.depth_profile": "640x480x30",
            "rgb_camera.color_profile":   "640x480x30",
        }],
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
                "camera_frame": "d435_link",
                "map_frame":    "map",
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
                "map_frame":                "map",
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
        DeclareLaunchArgument("model_path", default_value="yolo11n.pt",
                              description="YOLO model path (use yolo26n.engine on Jetson)"),
        DeclareLaunchArgument("rviz",       default_value="true",
                              description="Launch RViz"),
        DeclareLaunchArgument("slam_mode",  default_value="mapping",
                              description="'mapping' or 'localization'"),

        robot_state_publisher,
        realsense,
        slam,
        detection,
        human_obstacle,
        nav2,
        robot_node,
        rviz_node,
    ])
