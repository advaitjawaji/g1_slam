from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="yolo26n.pt"),
        DeclareLaunchArgument("conf",       default_value="0.25"),
        DeclareLaunchArgument("imgsz",      default_value="384"),

        Node(
            package="g1_detection",
            executable="detection_node",
            name="detection_node",
            output="screen",
            parameters=[{
                "model_path":    LaunchConfiguration("model_path"),
                "conf":          LaunchConfiguration("conf"),
                "imgsz":         LaunchConfiguration("imgsz"),
                "camera_frame":  "camera_depth_frame",
                "map_frame":     "map",
            }],
        ),
    ])
