from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip",   default_value="192.168.123.161"),
        DeclareLaunchArgument("robot_port", default_value="8080"),

        Node(
            package="g1_robot",
            executable="robot_node",
            name="robot_node",
            output="screen",
            parameters=[{
                "robot_ip":   LaunchConfiguration("robot_ip"),
                "robot_port": LaunchConfiguration("robot_port"),
            }],
        ),
    ])
