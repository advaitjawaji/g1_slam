#!/bin/bash
# Composite demo visualizer + recorder for the G1 human-avoidance stack.
#
# Renders the egocentric ZED view (bounding boxes + IDs + motion arrows) beside a
# robot-centric bird's-eye view (human predicted trails + Nav2 plan + traveled
# path + danger rings + status), publishes it on /viz/overlay, and records mp4.
#
# PREREQUISITE: the stack must already be running so the topics exist:
#   USE_ZED_ODOM=1 ./run_zed_e2e.sh          # ZED + odom + detection + Nav2
#   ros2 action send_goal /navigate_to_pose ...   # so /plan + /odom populate
# Then, in another terminal:
#   ./run_viz.sh                              # start rendering + recording
#
# View live:  ros2 run rqt_image_view rqt_image_view /viz/overlay
# Output:     demo_<timestamp>.mp4 in the repo root (override with RECORD_PATH=...)
set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
MODEL_PATH="${MODEL_PATH:-yolo26n.pt}"
RECORD_PATH="${RECORD_PATH:-}"        # "" -> auto demo_<ts>.mp4 in repo root

cd "$REPO_DIR"
source /opt/ros/${ROS_DISTRO_NAME}/setup.bash
source "$REPO_DIR/ros2_ws/install/setup.bash"
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

echo "=== VIZ + RECORD (Ctrl+C to stop and finalize the mp4) ==="
ARGS=(-p model_path:="$MODEL_PATH" -p record:=true)
# Only pass record_path when non-empty; an empty '-p record_path:=' fails to parse.
[ -n "$RECORD_PATH" ] && ARGS+=(-p record_path:="$RECORD_PATH")
exec ros2 run g1_detection viz_node --ros-args "${ARGS[@]}"
