"""
Simulation bringup: Gazebo + G1 + RTAB-Map SLAM + Human Detection

Usage:
    ros2 launch g1_bringup sim.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = LaunchConfiguration("model_path", default="yolo26n.pt")

    pkg_desc   = get_package_share_directory("g1_description")

    # Let Gazebo find package:// resources from the ROS2 install space
    gazebo_resource_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH",
        os.path.join(get_package_prefix("g1_description"), "share") + ":" +
        os.environ.get("GAZEBO_RESOURCE_PATH", ""),
    )
    pkg_gazebo = get_package_share_directory("gazebo_ros")

    urdf_path  = os.path.join(pkg_desc, "urdf", "g1_29dof.urdf")
    world_path = os.path.join(pkg_desc, "worlds", "test_room.world")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Launch Gazebo with test world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path, "verbose": "false", "gui": "false"}.items(),
    )

    # Publish robot description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Publish all joint states at 0 (no locomotion controller yet)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
    )

    # Spawn G1 at 0.8m height (pelvis above ground)
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "g1",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.8",
        ],
        output="screen",
    )

    # SLAM — wait for Gazebo sensors to come up
    slam = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("g1_slam"), "launch", "slam.launch.py"])
            ]),
            launch_arguments={
                "use_sim":       "true",
                "launch_camera": "false",
                "viz":           "true",
            }.items(),
        )],
    )

    # Detection disabled until numpy/cv_bridge conflict is resolved
    # TODO: re-enable once cv_bridge is rebuilt against numpy 2.x
    # detection = TimerAction(...)

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="yolo26n.pt"),
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        slam,
    ])
