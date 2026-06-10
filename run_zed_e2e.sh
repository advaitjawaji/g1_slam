#!/bin/bash
# End-to-end ZED + RTAB-Map + Detection + Nav2 + RViz launcher.
# Opens a tmux session 'g1_zed' with all components in split panes.
#
# Usage:
#   ./run_zed_e2e.sh            # launch full stack
#   ./run_zed_e2e.sh stop       # tear down the session
#   tmux attach -t g1_zed       # reattach later

set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$REPO_DIR/ros2_ws"
SESSION=g1_zed

# ── Helpers ───────────────────────────────────────────────────────────────
SOURCE_CMD="cd $REPO_DIR && source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash && export LD_LIBRARY_PATH=/usr/local/cuda-13.0/targets/x86_64-linux/lib:\$LD_LIBRARY_PATH"

# ── stop ──────────────────────────────────────────────────────────────────
if [[ "${1:-}" == "stop" ]]; then
    tmux kill-session -t $SESSION 2>/dev/null || true
    pkill -f zed_wrapper || true
    pkill -f rtabmap || true
    pkill -f nav2 || true
    pkill -f rviz2 || true
    pkill -f static_transform_publisher || true
    pkill -f robot_state_publisher || true
    pkill -f detection_node || true
    echo "Stopped."
    exit 0
fi

# ── Deps ──────────────────────────────────────────────────────────────────
command -v tmux >/dev/null || { echo "Installing tmux..."; sudo apt install -y tmux; }

URDF_PATH="$(source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash 2>/dev/null && ros2 pkg prefix g1_description 2>/dev/null)/share/g1_description/urdf/g1_29dof.urdf"
NAV2_PARAMS="$(source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash 2>/dev/null && ros2 pkg prefix g1_bringup 2>/dev/null)/share/g1_bringup/config/nav2_params_hw.yaml"

# ── Kill old session ──────────────────────────────────────────────────────
tmux kill-session -t $SESSION 2>/dev/null || true

echo ""
echo "============================================================"
echo "  G1 ZED end-to-end test"
echo "  Repo:        $REPO_DIR"
echo "  URDF:        $URDF_PATH"
echo "  Nav2 params: $NAV2_PARAMS"
echo "============================================================"
echo "Starting in 2 s (Ctrl+C to cancel)..."
sleep 2

# ── tmux layout: 2 windows ────────────────────────────────────────────────
#  Window 'core'   : zed | rtabmap | detection | nav2     (4 panes)
#  Window 'extras' : tf  | rsp     | rviz      | monitor  (4 panes)

tmux new-session -d -s $SESSION -x 220 -y 50 -n core
tmux split-window -h -t $SESSION:core
tmux split-window -v -t $SESSION:core.0
tmux split-window -v -t $SESSION:core.2

# Pane 0 - ZED
tmux send-keys -t $SESSION:core.0 \
    "$SOURCE_CMD && echo '=== ZED ===' && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i pos_tracking.pos_tracking_enabled:=false" Enter

# Pane 1 - RTAB-Map (wait for ZED topics)
tmux send-keys -t $SESSION:core.1 \
    "$SOURCE_CMD && sleep 10 && echo '=== RTAB-MAP ===' && ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/zed/zed_node/rgb/color/rect/image depth_topic:=/zed/zed_node/depth/depth_registered camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info frame_id:=zed_camera_link approx_sync:=true approx_sync_max_interval:=0.02 qos:=2 rtabmap_viz:=false rviz:=false" Enter

# Pane 2 - Detection
tmux send-keys -t $SESSION:core.2 \
    "$SOURCE_CMD && sleep 12 && echo '=== DETECTION ===' && ros2 run g1_detection detection_node --ros-args -p model_path:=yolo26n.pt -p color_topic:=/zed/zed_node/rgb/color/rect/image -p depth_topic:=/zed/zed_node/depth/depth_registered -p camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info -p camera_frame:=zed_left_camera_frame" Enter

# Pane 3 - Nav2 + auto-activate
tmux send-keys -t $SESSION:core.3 \
    "$SOURCE_CMD && sleep 14 && echo '=== NAV2 ===' && (ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=$NAV2_PARAMS) & sleep 12 && ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 2}' && wait" Enter

# ── Window 'extras' ───────────────────────────────────────────────────────
tmux new-window -t $SESSION -n extras
tmux split-window -h -t $SESSION:extras
tmux split-window -v -t $SESSION:extras.0
tmux split-window -v -t $SESSION:extras.2

# Pane 0 - static TF pelvis
tmux send-keys -t $SESSION:extras.0 \
    "$SOURCE_CMD && sleep 6 && echo '=== STATIC TF ===' && ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id zed_camera_link --child-frame-id pelvis" Enter

# Pane 1 - robot_state_publisher
tmux send-keys -t $SESSION:extras.1 \
    "$SOURCE_CMD && sleep 6 && echo '=== ROBOT STATE PUBLISHER ===' && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(cat $URDF_PATH)\"" Enter

# Pane 2 - RViz
tmux send-keys -t $SESSION:extras.2 \
    "$SOURCE_CMD && sleep 16 && echo '=== RVIZ ===' && rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz" Enter

# Pane 3 - monitor
tmux send-keys -t $SESSION:extras.3 \
    "$SOURCE_CMD && sleep 18 && echo '=== MONITOR ===' && echo 'Commands:' && echo '  ros2 topic hz /cmd_vel' && echo '  ros2 topic echo /g1/human_cmd' && echo '  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...' && bash" Enter

# Focus core window
tmux select-window -t $SESSION:core

echo ""
echo "Attach with:    tmux attach -t $SESSION"
echo "Stop with:      ./run_zed_e2e.sh stop"
echo ""
echo "Send a goal (from any sourced shell):"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}\" --feedback"
echo ""

# Auto-attach if interactive
if [[ -t 1 ]]; then
    sleep 1
    tmux attach -t $SESSION
fi
