from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rtabmap_config = PathJoinSubstitution([FindPackageShare("g1_slam"), "config", "rtabmap.yaml"])
    map_file = LaunchConfiguration("map", default="")

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_config, {"Mem/IncrementalMemory": "false"}],
        remappings=[
            ("rgb/image",         "/camera/color/image_raw"),
            ("rgb/camera_info",   "/camera/color/camera_info"),
            ("depth/image",       "/camera/aligned_depth_to_color/image_raw"),
            ("imu",               "/camera/imu"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=""),
        rtabmap_node,
    ])
