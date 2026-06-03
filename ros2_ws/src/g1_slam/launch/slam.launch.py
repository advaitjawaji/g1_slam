from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim", default="false")
    rtabmap_config = PathJoinSubstitution([FindPackageShare("g1_slam"), "config", "rtabmap.yaml"])

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        parameters=[{
            "enable_color": True,
            "enable_depth": True,
            "enable_gyro": True,
            "enable_accel": True,
            "unite_imu_method": "linear_interpolation",
            "align_depth.enable": True,
            "depth_module.depth_profile": "640x480x30",
            "rgb_camera.color_profile": "640x480x30",
        }],
        condition=IfCondition(LaunchConfiguration("launch_camera", default="true")),
    )

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_config],
        remappings=[
            ("rgb/image",         "/camera/color/image_raw"),
            ("rgb/camera_info",   "/camera/color/camera_info"),
            ("depth/image",       "/camera/aligned_depth_to_color/image_raw"),
            ("imu",               "/imu_in_torso/data"),
        ],
        arguments=["--delete_db_on_start"],
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[rtabmap_config],
        remappings=[
            ("rgb/image",         "/camera/color/image_raw"),
            ("rgb/camera_info",   "/camera/color/camera_info"),
            ("depth/image",       "/camera/aligned_depth_to_color/image_raw"),
        ],
        condition=IfCondition(LaunchConfiguration("viz", default="true")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim",       default_value="false"),
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("viz",           default_value="true"),
        realsense_node,
        rtabmap_node,
        rtabmap_viz,
    ])
