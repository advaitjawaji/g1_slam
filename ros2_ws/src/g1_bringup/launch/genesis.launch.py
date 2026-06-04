"""
Genesis simulation bringup — full pipeline validation.

Launches the ROS2 stack (SLAM + detection + Nav2) that connects to the
Genesis bridge node running in a separate terminal.

Usage:
    # Terminal 1 — Genesis bridge (Python, not ROS2)
    cd ~/Desktop/g1_slam
    python3 genesis_sim/genesis_node.py

    # Terminal 2 — ROS2 stack
    ros2 launch g1_bringup genesis.launch.py

    # Terminal 3 — drive robot or set Nav2 goals
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    nav2_params = os.path.join(pkg_bringup, "config", "nav2_params.yaml")
    rviz_config = os.path.join(pkg_bringup, "config", "demo.rviz")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ── Robot description ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    # ── RTAB-Map SLAM ─────────────────────────────────────────────────────
    # Camera data comes from Genesis bridge node
    rtabmap_odom = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rtabmap_odom",
        output="screen",
        parameters=[{
            "frame_id":       "pelvis",
            "odom_frame_id":  "odom",
            "publish_tf":     True,
            "use_sim_time":   False,
        }],
        remappings=[
            ("rgb/image",       "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("depth/image",     "/camera/aligned_depth_to_color/image_raw"),
            ("odom",            "/odom/visual"),
        ],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            rtabmap_cfg,
            {"use_sim_time": False},
        ],
        remappings=[
            ("rgb/image",       "/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("depth/image",     "/camera/aligned_depth_to_color/image_raw"),
            ("odom",            "/odom/raw"),
        ],
        arguments=["--delete_db_on_start"],
    )

    # ── Human detection ───────────────────────────────────────────────────
    # NOTE: detection_node (YOLO) is NOT launched in Genesis mode.
    # genesis_node.py publishes ground truth /humans/markers directly,
    # bypassing YOLO since Genesis human actors are simple cylinders.
    # human_obstacle_node still runs to inject those positions into Nav2.

    human_obstacle = TimerAction(
        period=6.0,
        actions=[Node(
            package="g1_detection",
            executable="human_obstacle_node",
            name="human_obstacle_node",
            output="screen",
        )],
    )

    # ── Nav2 — use nav2_bringup which handles lifecycle correctly ────────
    # Wait 25s so RTAB-Map has time to build a map before Nav2 configures
    nav2 = TimerAction(
        period=25.0,
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

    # ── RViz ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="yolo11n.pt"),
        DeclareLaunchArgument("rviz",       default_value="true"),

        LogInfo(msg="\n" + "="*60 +
                "\n  GENESIS SIMULATION MODE" +
                "\n  Make sure genesis_node.py is running in another terminal:" +
                "\n  python3 genesis_sim/genesis_node.py" +
                "\n" + "="*60 + "\n"),

        robot_state_publisher,
        joint_state_publisher,
        rtabmap_odom,
        rtabmap,
        human_obstacle,
        nav2,
        rviz_node,
    ])
