"""Tier 3 hardware bringup — autonomous go-to-pose WITHOUT a camera.

Localization comes from the G1's own motion estimate via `odom_bridge`
(SDK SportModeState if the firmware publishes it, else IMU-yaw + cmd_vel,
else pure dead-reckoning). A static identity map->odom makes the robot start
at the origin. Nav2 plans through free space (sim/permissive params) and
`robot_node` drives the legs over the SDK.

This is deliberately a strict SUBSET of hardware.launch.py: the same Nav2,
robot_node, odom_bridge, frames and `map`-frame goals. Adding the ZED later is
purely additive — drop the static map->odom and add ZED + RTAB-Map (map->odom)
+ detection. Nav2 / robot_node / odom_bridge stay identical.

  ⚠️  NO obstacle sensing and NO human detection here. The robot walks BLIND
      toward the goal. Run only in a clear, supervised space with the e-stop ready.

Usage:
    ros2 launch g1_bringup nav_tier3.launch.py net_iface:=enp4s0
    # then send a goal in the 'map' frame, e.g. 2 m ahead:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback

Arguments:
    net_iface — host NIC on the robot subnet for DDS (e.g. enp4s0); empty=auto
    rviz      — launch RViz (default: true)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    net_iface = LaunchConfiguration("net_iface", default="")
    rviz      = LaunchConfiguration("rviz",      default="true")

    pkg_desc    = get_package_share_directory("g1_description")
    pkg_bringup = get_package_share_directory("g1_bringup")

    urdf_path   = os.path.join(pkg_desc, "urdf", "g1_29dof.urdf")
    nav2_params = os.path.join(pkg_bringup, "config", "nav2_params_sim.yaml")
    rviz_config = os.path.join(pkg_bringup, "config", "demo.rviz")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ── Robot description (URDF) → body TFs + RViz model ──────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # ── Static identity map → odom (no SLAM to provide it) ────────────────
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--yaw", "0", "--pitch", "0", "--roll", "0",
                   "--frame-id", "map", "--child-frame-id", "odom"],
    )

    # ── Odometry from the robot (odom → pelvis). Publish as /odom/raw to
    #    match controller_server's odom_topic in nav2_params_sim.yaml. ─────
    odom_bridge = Node(
        package="g1_robot",
        executable="odom_bridge",
        name="odom_bridge",
        output="screen",
        parameters=[{"net_iface": net_iface}],
        remappings=[("/odom", "/odom/raw")],
    )

    # ── Nav2 (free-space planning; no obstacle sensors here) ──────────────
    nav2 = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare("nav2_bringup"),
                                      "launch", "navigation_launch.py"])
            ]),
            launch_arguments={
                "use_sim_time": "false",
                "params_file":  nav2_params,
            }.items(),
        )],
    )

    # ── Force Nav2 lifecycle active (autostart is flaky) ──────────────────
    nav2_activate = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "service", "call",
                 "/lifecycle_manager_navigation/manage_nodes",
                 "nav2_msgs/srv/ManageLifecycleNodes", "{command: 2}"],
            output="screen",
        )],
    )

    # ── G1 locomotion node (/cmd_vel + /g1/human_cmd → SDK Move) ──────────
    robot_node = Node(
        package="g1_robot",
        executable="robot_node",
        name="robot_node",
        output="screen",
        parameters=[{"net_iface": net_iface, "robot_port": 8080}],
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("net_iface", default_value="",
                              description="Host NIC on robot subnet for DDS (e.g. enp4s0); empty=auto"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz"),

        robot_state_publisher,
        map_to_odom,
        odom_bridge,
        nav2,
        nav2_activate,
        robot_node,
        rviz_node,
    ])
