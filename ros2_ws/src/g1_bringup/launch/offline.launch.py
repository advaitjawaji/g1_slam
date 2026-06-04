"""
Offline pipeline test using a pre-recorded rosbag.
Tests: SLAM, human detection, human avoidance logic, Nav2 path planning.
Does NOT test: actual G1 walking.

Usage:
    # Terminal 1 — launch pipeline
    ros2 launch g1_bringup offline.launch.py bag:=/path/to/lab_test

    # Terminal 2 — play bag (after pipeline is up)
    ros2 bag play /path/to/lab_test --clock --rate 1.0

    # Terminal 3 — set navigation goals in RViz, watch /cmd_vel
    ros2 topic echo /cmd_vel

Arguments:
    bag         Path to rosbag directory (for display only — play bag manually)
    model_path  YOLO model file (default: yolo11n.pt)
    rviz        Launch RViz (default: true)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = LaunchConfiguration("model_path", default="yolo11n.pt")
    rviz       = LaunchConfiguration("rviz",       default="true")

    pkg_desc    = get_package_share_directory("g1_description")
    pkg_bringup = get_package_share_directory("g1_bringup")

    urdf_path   = os.path.join(pkg_desc, "urdf", "g1_29dof.urdf")
    rtabmap_cfg = PathJoinSubstitution([FindPackageShare("g1_slam"), "config", "rtabmap.yaml"])
    nav2_params = os.path.join(pkg_bringup, "config", "nav2_params_sim.yaml")
    rviz_config = os.path.join(pkg_bringup, "config", "offline.rviz")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ── Robot description ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time":      True,   # sync with bag /clock
        }],
    )

    # Publish all joints at zero (no locomotion in offline mode)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
    )

    # ── RTAB-Map SLAM ─────────────────────────────────────────────────────
    # Reads camera topics from the bag, builds map
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            rtabmap_cfg,
            {"use_sim_time": True},
        ],
        remappings=[
            ("rgb/image",       "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("depth/image",     "/camera/aligned_depth_to_color/image_raw"),
            ("imu",             "/imu_in_torso/data"),
            ("odom",            "/odom/bag"),   # odom from bag if recorded, else RTAB-Map generates it
        ],
        arguments=["--delete_db_on_start"],
    )

    # RTAB-Map visual odometry (generates odom when no wheel odom in bag)
    rtabmap_odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rtabmap_odom",
        output="screen",
        parameters=[{
            "use_sim_time":      True,
            "frame_id":          "pelvis",
            "odom_frame_id":     "odom",
            "publish_tf":        True,
            "wait_for_transform": 0.2,
        }],
        remappings=[
            ("rgb/image",       "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("depth/image",     "/camera/aligned_depth_to_color/image_raw"),
            ("odom",            "/odom/visual"),
        ],
    )

    # ── Human detection ───────────────────────────────────────────────────
    detection = TimerAction(
        period=5.0,   # wait for SLAM to initialise
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
                "use_sim_time": True,
            }],
        )],
    )

    # ── Human obstacle injection ──────────────────────────────────────────
    human_obstacle = TimerAction(
        period=5.0,
        actions=[Node(
            package="g1_detection",
            executable="human_obstacle_node",
            name="human_obstacle_node",
            output="screen",
            parameters=[{
                "map_frame":                "map",
                "prediction_horizon_steps": 10,
                "use_sim_time":             True,
            }],
        )],
    )

    # ── Nav2 navigation stack ─────────────────────────────────────────────
    nav2_nodes = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
                remappings=[("cmd_vel", "/cmd_vel")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time":  True,
                    "autostart":     True,
                    "node_names":    [
                        "controller_server",
                        "planner_server",
                        "bt_navigator",
                    ],
                }],
            ),
        ],
    )

    # ── cmd_vel monitor — prints what robot WOULD do ──────────────────────
    cmd_vel_monitor = Node(
        package="topic_tools",
        executable="echo",
        name="cmd_vel_monitor",
        arguments=["/cmd_vel", "geometry_msgs/msg/Twist"],
        output="screen",
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="yolo11n.pt",
                              description="YOLO model path"),
        DeclareLaunchArgument("rviz",       default_value="true",
                              description="Launch RViz"),

        LogInfo(msg="\n" + "="*60 +
                "\n  OFFLINE PIPELINE TEST" +
                "\n  After this launches, in a NEW terminal run:" +
                "\n  ros2 bag play <your_bag_path> --clock --rate 1.0" +
                "\n" + "="*60 + "\n"),

        robot_state_publisher,
        joint_state_publisher,
        rtabmap_odom,
        rtabmap,
        detection,
        human_obstacle,
        nav2_nodes,
        rviz_node,
    ])
