#!/bin/bash
# Hands-free demo tour for recording.
# Sends a sequence of navigation goals that route the G1 across the room and
# through the human patrol paths so SLAM mapping, human detection, and
# avoidance are all visible. Run AFTER ./run_genesis.sh and ./activate_nav2.sh.
#
# Usage:  ./demo_tour.sh

source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "$0")/ros2_ws" && pwd)/install/setup.bash"

send_goal () {
    local x=$1 y=$2 label=$3
    echo ""
    echo "────────────────────────────────────────────"
    echo "  Goal: $label   ->  ($x, $y)"
    echo "────────────────────────────────────────────"
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {w: 1.0}}}}" \
      2>/dev/null | grep -E "status"
    sleep 2   # brief pause so the stop/settle reads clearly on video
}

echo "Starting demo tour in 3 seconds — start your screen recording now."
sleep 3

# Human 1 patrols x=2 (y=-3..3); Human 2 patrols (-2,3)->(3,-1).
# These goals weave through both so avoidance is on camera.
send_goal  3.0   0.0   "cross room (through human 1 path)"
send_goal  0.0   3.0   "up the left side"
send_goal -3.0   2.0   "across to far corner (through human 2 path)"
send_goal  2.0  -2.5   "diagonal back across"
send_goal  0.0   0.0   "return to start"

echo ""
echo "Demo tour complete."
