"""
Hardware bringup: RealSense + RTAB-Map SLAM + Human Detection + G1 SDK

Usage:
    ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip   = LaunchConfiguration("robot_ip",   default="192.168.123.161")
    model_path = LaunchConfiguration("model_path", default="yolo26n.pt")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("g1_slam"), "launch", "slam.launch.py"])
        ]),
        launch_arguments={
            "use_sim":       "false",
            "launch_camera": "true",
            "viz":           "true",
        }.items(),
    )

    detection = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("g1_detection"), "launch", "detection.launch.py"])
            ]),
            launch_arguments={"model_path": model_path}.items(),
        )],
    )

    robot = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("g1_robot"), "launch", "robot.launch.py"])
            ]),
            launch_arguments={"robot_ip": robot_ip}.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip",   default_value="192.168.123.161"),
        DeclareLaunchArgument("model_path", default_value="yolo26n.pt"),
        slam,
        detection,
        robot,
    ])
