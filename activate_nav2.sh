#!/bin/bash
# Manually activate all Nav2 nodes in order.
# Run this after ./run_genesis.sh, once the simulation is up (~30 seconds).

source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "$0")/ros2_ws" && pwd)/install/setup.bash"

echo "Activating Nav2 nodes..."

lifecycle_set() {
    local node=$1 transition=$2
    timeout 8 ros2 lifecycle set /$node $transition 2>/dev/null || echo "timeout/skip"
}

for node in controller_server smoother_server planner_server behavior_server waypoint_follower velocity_smoother; do
    echo -n "  $node configure: "
    lifecycle_set $node configure
    echo -n "  $node activate:  "
    lifecycle_set $node activate
done

echo -n "  bt_navigator configure: "
lifecycle_set bt_navigator configure
echo -n "  bt_navigator activate:  "
lifecycle_set bt_navigator activate
echo "done"

echo ""
echo "Nav2 active. Set a goal in RViz with '2D Goal Pose' or:"
echo "  ros2 topic pub /goal_pose geometry_msgs/PoseStamped \\"
echo "    \"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}\" --once"
