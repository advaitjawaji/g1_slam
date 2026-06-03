#!/bin/bash
# Offline pipeline test using a pre-recorded rosbag
#
# Usage:
#   ./test_offline.sh /path/to/lab_test_bag
#   ./test_offline.sh /path/to/lab_test_bag 0.5   # half speed playback

BAG_PATH=${1:-""}
RATE=${2:-"1.0"}

WS_DIR="$(cd "$(dirname "$0")/ros2_ws" && pwd)"

if [ -z "$BAG_PATH" ]; then
    echo "Usage: $0 <bag_path> [rate]"
    echo "Example: $0 ~/bags/lab_test 0.5"
    exit 1
fi

if [ ! -d "$BAG_PATH" ]; then
    echo "Error: bag not found at $BAG_PATH"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

echo ""
echo "============================================================"
echo "  G1 Human Avoidance — Offline Pipeline Test"
echo "  Bag: $BAG_PATH"
echo "  Rate: ${RATE}x"
echo "============================================================"
echo ""
echo "Launching pipeline in 3 seconds..."
sleep 3

# Launch pipeline in background
ros2 launch g1_bringup offline.launch.py &
LAUNCH_PID=$!

echo ""
echo "Waiting 8 seconds for nodes to start..."
sleep 8

echo ""
echo "Playing bag at ${RATE}x speed..."
ros2 bag play "$BAG_PATH" --clock --rate "$RATE" --loop

# Cleanup on exit
kill $LAUNCH_PID 2>/dev/null
