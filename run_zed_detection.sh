#!/bin/bash
# ZED 2 camera-only human-detection test — NO robot, NO SLAM, NO Nav2.
# The minimal loop that closes STATE.md §9 step 1: real YOLO detection on a real
# ZED camera. Success = /g1/human_cmd flips NORMAL_OPERATION <-> SLOW_DOWN <-> STOP
# as you walk toward/away from the camera.
#
# Target platform: Jetson AGX Thor, Ubuntu 24.04, ROS 2 Jazzy, ZED SDK >=5.1.
# (The Humble container cannot run the ZED path — glibc 2.35 vs the 24.04 host's
#  2.39; see docker/README.md. This runs natively on the host.)
#
# Usage:
#   ./run_zed_detection.sh          # launch the 3-pane test
#   ./run_zed_detection.sh stop     # tear down
#   tmux attach -t g1_zed_det       # reattach later
set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$REPO_DIR/ros2_ws"
SESSION=g1_zed_det

# ── Tunables — VERIFY these against your setup ────────────────────────────
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
CAMERA_MODEL="${CAMERA_MODEL:-zed2i}"       # confirmed ZED 2i via ZED_Diagnostic 2026-07-23
MODEL_PATH="${MODEL_PATH:-yolo26n.pt}"      # auto-downloads if absent (needs internet)

# ZED topic names. Confirm with:  ros2 topic list | grep zed
COLOR_TOPIC="${COLOR_TOPIC:-/zed/zed_node/rgb/color/rect/image}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/zed/zed_node/depth/depth_registered}"
INFO_TOPIC="${INFO_TOPIC:-/zed/zed_node/rgb/color/rect/camera_info}"
CAMERA_FRAME="${CAMERA_FRAME:-zed_left_camera_frame}"

# Source ROS + workspace + CUDA libs (arch-agnostic: /usr/local/cuda is the
# JetPack symlink; no hardcoded x86_64 path).
SOURCE_CMD="cd $REPO_DIR \
  && source /opt/ros/${ROS_DISTRO_NAME}/setup.bash \
  && source $WS_DIR/install/setup.bash \
  && export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH"

# ── stop ──────────────────────────────────────────────────────────────────
if [[ "${1:-}" == "stop" ]]; then
    tmux kill-session -t $SESSION 2>/dev/null || true
    pkill -f zed_wrapper || true
    pkill -f detection_node || true
    echo "Stopped."
    exit 0
fi

command -v tmux >/dev/null || { echo "Installing tmux..."; sudo apt install -y tmux; }

tmux kill-session -t $SESSION 2>/dev/null || true

echo ""
echo "============================================================"
echo "  ZED camera-only detection test"
echo "  ROS distro:   $ROS_DISTRO_NAME"
echo "  Camera:       $CAMERA_MODEL"
echo "  Color topic:  $COLOR_TOPIC"
echo "  Model:        $MODEL_PATH"
echo "============================================================"
echo "Starting in 2 s (Ctrl+C to cancel)..."
sleep 2

# ── tmux: 3 panes — ZED | detection | /g1/human_cmd ───────────────────────
tmux new-session -d -s $SESSION -x 220 -y 50 -n det
tmux split-window -h -t $SESSION:det
tmux split-window -v -t $SESSION:det.1

# Pane 0 — ZED wrapper
tmux send-keys -t $SESSION:det.0 \
    "$SOURCE_CMD && echo '=== ZED ===' && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=$CAMERA_MODEL pos_tracking.pos_tracking_enabled:=false" Enter

# Pane 1 — detection node (map_frame == camera_frame so it needs no TF tree)
tmux send-keys -t $SESSION:det.1 \
    "$SOURCE_CMD && sleep 12 && echo '=== DETECTION ===' && ros2 run g1_detection detection_node --ros-args \
      -p model_path:=$MODEL_PATH \
      -p color_topic:=$COLOR_TOPIC \
      -p depth_topic:=$DEPTH_TOPIC \
      -p camera_info_topic:=$INFO_TOPIC \
      -p camera_frame:=$CAMERA_FRAME \
      -p map_frame:=$CAMERA_FRAME" Enter

# Pane 2 — the signal you care about
tmux send-keys -t $SESSION:det.2 \
    "$SOURCE_CMD && sleep 15 && echo '=== /g1/human_cmd (walk toward the camera) ===' && ros2 topic echo /g1/human_cmd" Enter

tmux select-window -t $SESSION:det
echo ""
echo "Attach:  tmux attach -t $SESSION       Stop:  ./run_zed_detection.sh stop"
echo ""

if [[ -t 1 ]]; then
    sleep 1
    tmux attach -t $SESSION
fi
