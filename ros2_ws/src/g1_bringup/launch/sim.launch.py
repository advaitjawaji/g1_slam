"""
Simulation bringup: Gazebo + G1 + RTAB-Map SLAM + Human Detection

Usage:
    ros2 launch g1_bringup sim.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world", default="empty.world")
    model_path = LaunchConfiguration("model_path", default="yolo26n.pt")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ]),
        launch_arguments={"world": world}.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("g1_description"), "launch", "display.launch.py"])
        ]),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("g1_slam"), "launch", "slam.launch.py"])
        ]),
        launch_arguments={
            "use_sim": "true",
            "launch_camera": "false",  # camera provided by Gazebo
            "viz": "true",
        }.items(),
    )

    detection = TimerAction(
        period=5.0,  # wait for SLAM to initialise
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("g1_detection"), "launch", "detection.launch.py"])
            ]),
            launch_arguments={"model_path": model_path}.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("world",      default_value="empty.world"),
        DeclareLaunchArgument("model_path", default_value="yolo26n.pt"),
        gazebo,
        robot_state_publisher,
        slam,
        detection,
    ])
