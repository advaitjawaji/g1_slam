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

# Drive the real G1? OFF by default so this stays a perception-only test and
# never surprise-walks the robot. Enable explicitly:
#   WITH_ROBOT=1 NET_IFACE=enp4s0 ./run_zed_e2e.sh
# Robot must already be STANDING (robot_node does not auto-stand). e-stop ready.
WITH_ROBOT="${WITH_ROBOT:-0}"
NET_IFACE="${NET_IFACE:-}"

# Localization source. Default = RTAB-Map visual odometry (RGB-D SLAM).
# USE_ZED_ODOM=1 instead uses the ZED 2i's own visual-inertial odometry (VIO),
# smoother/IMU-fused and better at tracking the walking G1. It disables RTAB-Map
# and relays ZED odom onto /odom (Nav2's odom_topic in nav2_params_hw.yaml).
# Both modes now navigate in the `odom` frame — no map frame is published.
#   USE_ZED_ODOM=1 ./run_zed_e2e.sh
# ⚠️ First lab run: verify the TF chain is a single tree odom->zed_camera_link->
#    pelvis with `ros2 run tf2_tools view_frames` (no frame with two parents),
#    and that the static ZED->pelvis mount offset matches the real hardware.
#    In RViz set Fixed Frame = `odom` (there is no `map` frame in this wiring).
USE_ZED_ODOM="${USE_ZED_ODOM:-0}"

# Demo overlay: egocentric ZED video annotated with Human_dtp bounding boxes,
# human trajectory forecasts, the Nav2 plan and the path actually walked. Writes
# an MP4 per run. Off with OVERLAY=0; recording off with RECORD=0.
#   DEMO_DIR=~/demos ./run_zed_e2e.sh
# View live:  ros2 run rqt_image_view rqt_image_view /g1/overlay/image
OVERLAY="${OVERLAY:-1}"
RECORD="${RECORD:-1}"
DEMO_DIR="${DEMO_DIR:-$HOME/g1_demos}"

# ── Helpers ───────────────────────────────────────────────────────────────
SOURCE_CMD="cd $REPO_DIR && source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash && export LD_LIBRARY_PATH=/usr/local/cuda-13.0/targets/x86_64-linux/lib:\$LD_LIBRARY_PATH"

# ── stop ──────────────────────────────────────────────────────────────────
if [[ "${1:-}" == "stop" ]]; then
    # ORDER MATTERS. The MP4 is only finalised when overlay_node runs its
    # shutdown handler, so it must be SIGINT'd and given time to exit BEFORE
    # tmux kill-session SIGHUPs every pane. Killing the session first leaves an
    # unplayable file.
    if pgrep -f overlay_node >/dev/null 2>&1; then
        echo "Flushing demo video..."
        pkill -INT -f overlay_node || true
        for _ in $(seq 1 40); do            # up to 8 s
            pgrep -f overlay_node >/dev/null 2>&1 || break
            sleep 0.2
        done
        pgrep -f overlay_node >/dev/null 2>&1 \
            && { echo "  overlay_node did not exit; forcing (MP4 may be truncated)."; \
                 pkill -KILL -f overlay_node || true; } \
            || echo "  done."
    fi

    tmux kill-session -t $SESSION 2>/dev/null || true
    pkill -f zed_wrapper || true
    pkill -f rtabmap || true
    pkill -f nav2 || true
    pkill -f rviz2 || true
    pkill -f static_transform_publisher || true
    pkill -f robot_state_publisher || true
    pkill -f detection_node || true
    pkill -f human_obstacle_node || true
    pkill -f robot_node || true
    pkill -f "relay /zed/zed_node/odom" || true
    echo "Stopped."
    exit 0
fi

# ── Deps ──────────────────────────────────────────────────────────────────
command -v tmux >/dev/null || { echo "Installing tmux..."; sudo apt install -y tmux; }

# ZED-odom mode relays ZED VIO onto /rtabmap/odom via topic_tools relay.
if [[ "$USE_ZED_ODOM" == "1" ]]; then
    ( source /opt/ros/humble/setup.bash 2>/dev/null; \
      ros2 pkg executables topic_tools 2>/dev/null | grep -q relay ) \
      || { echo "Installing ros-humble-topic-tools (ZED-odom relay)..."; sudo apt install -y ros-humble-topic-tools; }
fi

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
if [[ "$WITH_ROBOT" == "1" ]]; then
    echo "  Robot:       DRIVING (robot_node, iface='${NET_IFACE:-auto}') — robot must be STANDING, e-stop ready"
else
    echo "  Robot:       perception-only (no robot_node). Enable: WITH_ROBOT=1 NET_IFACE=<nic> $0"
fi
if [[ "$USE_ZED_ODOM" == "1" ]]; then
    echo "  Odometry:    ZED 2i VIO -> /odom (RTAB-Map OFF, odom-frame nav, no map) — verify: ros2 run tf2_tools view_frames"
else
    echo "  Odometry:    RTAB-Map VO -> /odom. Use ZED VIO instead: USE_ZED_ODOM=1 $0"
fi
echo "============================================================"
echo "Starting in 2 s (Ctrl+C to cancel)..."
sleep 2

# ── Localization mode → ZED launch args + which node owns map->odom ─────────
if [[ "$USE_ZED_ODOM" == "1" ]]; then
    # ZED VIO owns odom->zed_camera_link (pos_tracking on via config default;
    # the wrapper has no base_frame param, so odom is always referenced to
    # zed_camera_link). No map frame: we navigate in `odom` (nav2_params_hw
    # global_frame=odom), so publish_map_tf:=false. publish_tf:=true emits the
    # odom->camera TF. (publish_tf/publish_map_tf are the only valid launch args.)
    ZED_TRACK_ARGS="publish_tf:=true publish_map_tf:=false"
else
    # RTAB-Map owns odom->base + map->odom; keep ZED from publishing any TF.
    ZED_TRACK_ARGS="publish_tf:=false"
fi

# ── tmux layout: 2 windows ────────────────────────────────────────────────
#  Window 'core'   : zed | rtabmap | detection | nav2     (4 panes)
#  Window 'extras' : tf  | rsp     | rviz      | monitor  (4 panes)

tmux new-session -d -s $SESSION -x 220 -y 50 -n core
tmux split-window -h -t $SESSION:core
tmux split-window -v -t $SESSION:core.0
tmux split-window -v -t $SESSION:core.2

# Pane 0 - ZED
tmux send-keys -t $SESSION:core.0 \
    "$SOURCE_CMD && echo '=== ZED ===' && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i $ZED_TRACK_ARGS" Enter

# Pane 1 - odom source: RTAB-Map VO (default) OR a relay of ZED VIO odom
if [[ "$USE_ZED_ODOM" == "1" ]]; then
    # ZED VIO owns odom->zed_camera_link (->pelvis via the static TF below).
    # Relay ZED odom onto /odom — the topic Nav2 reads (nav2_params_hw
    # odom_topic: /odom). Needs ros-humble-topic-tools.
    tmux send-keys -t $SESSION:core.1 \
        "$SOURCE_CMD && sleep 12 && echo '=== ZED-VIO ODOM (RTAB-Map OFF; relay ZED odom -> /odom) ===' && ros2 run topic_tools relay /zed/zed_node/odom /odom" Enter
else
    # RTAB-Map publishes odometry directly on /odom (Nav2's odom_topic).
    tmux send-keys -t $SESSION:core.1 \
        "$SOURCE_CMD && sleep 10 && echo '=== RTAB-MAP (odom -> /odom) ===' && ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/zed/zed_node/rgb/color/rect/image depth_topic:=/zed/zed_node/depth/depth_registered camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info frame_id:=zed_camera_link odom_topic:=/odom approx_sync:=true approx_sync_max_interval:=0.02 qos:=2 rtabmap_viz:=false rviz:=false" Enter
fi

# Pane 2 - Detection
tmux send-keys -t $SESSION:core.2 \
    "$SOURCE_CMD && sleep 12 && echo '=== DETECTION ===' && ros2 run g1_detection detection_node --ros-args -p model_path:=yolo26n.pt -p color_topic:=/zed/zed_node/rgb/color/rect/image -p depth_topic:=/zed/zed_node/depth/depth_registered -p camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info -p camera_frame:=zed_left_camera_frame -p world_frame:=odom" Enter

# Pane 3 - Nav2 + auto-activate
tmux send-keys -t $SESSION:core.3 \
    "$SOURCE_CMD && sleep 14 && echo '=== NAV2 ===' && (ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=$NAV2_PARAMS) & sleep 12 && ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 2}' && wait" Enter

# ── Window 'extras' ───────────────────────────────────────────────────────
tmux new-window -t $SESSION -n extras
tmux split-window -h -t $SESSION:extras
tmux split-window -v -t $SESSION:extras.0
tmux split-window -v -t $SESSION:extras.2

# Pane 0 - static TF: ZED head mount (zed_camera_link -> pelvis).
# Cam is 5cm fwd / 40cm up / ~17.5° down from pelvis; these are the INVERSE
# (camera->pelvis) values: t = -Ry(-0.3054)*(0.05,0,0.40) = (0.0726,0,-0.3965),
# rot pitch -0.3054. Must match hardware.launch.py zed_to_pelvis_tf.
tmux send-keys -t $SESSION:extras.0 \
    "$SOURCE_CMD && sleep 6 && echo '=== STATIC TF (zed_camera_link -> pelvis) ===' && ros2 run tf2_ros static_transform_publisher --x 0.0726 --y 0.0 --z -0.3965 --yaw 0 --pitch -0.3054 --roll 0 --frame-id zed_camera_link --child-frame-id pelvis" Enter

# Pane 1 - robot_state_publisher
tmux send-keys -t $SESSION:extras.1 \
    "$SOURCE_CMD && sleep 6 && echo '=== ROBOT STATE PUBLISHER ===' && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(cat $URDF_PATH)\"" Enter

# Pane 2 - RViz
tmux send-keys -t $SESSION:extras.2 \
    "$SOURCE_CMD && sleep 16 && echo '=== RVIZ ===' && rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz" Enter

# Pane 3 - monitor
tmux send-keys -t $SESSION:extras.3 \
    "$SOURCE_CMD && sleep 18 && echo '=== MONITOR ===' && echo 'Commands:' && echo '  ros2 topic hz /cmd_vel' && echo '  ros2 topic echo /g1/human_cmd' && echo '  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...' && bash" Enter

# ── Window 'ctrl' : human-obstacle injector (+ optional robot driver) ──────
tmux new-window -t $SESSION -n ctrl

# Pane 0 - human_obstacle_node : /humans/markers -> /human_obstacle_cloud
#          (perception only, always on; costmap source is still opt-in in YAML)
tmux send-keys -t $SESSION:ctrl.0 \
    "$SOURCE_CMD && sleep 14 && echo '=== HUMAN OBSTACLE (PointCloud2) ===' && ros2 run g1_detection human_obstacle_node" Enter

if [[ "$WITH_ROBOT" == "1" ]]; then
    tmux split-window -v -t $SESSION:ctrl.0
    # Pane 1 - robot_node : drives the real G1 from /cmd_vel.
    tmux send-keys -t $SESSION:ctrl.1 \
        "$SOURCE_CMD && sleep 8 && echo '=== ROBOT NODE — DRIVES THE G1 (robot must be STANDING, e-stop ready) ===' && ros2 run g1_robot robot_node --ros-args -p net_iface:=$NET_IFACE" Enter
fi

# ── Window 'demo' : the annotated egocentric video ────────────────────────
if [[ "$OVERLAY" == "1" ]]; then
    mkdir -p "$DEMO_DIR"
    DEMO_MP4="$DEMO_DIR/g1_demo_$(date +%Y%m%d_%H%M%S).mp4"
    tmux new-window -t $SESSION -n demo
    # Starts after detection so intrinsics + the first /g1/detections have
    # landed; it renders regardless, so a late detector just means the opening
    # seconds have no boxes.
    tmux send-keys -t $SESSION:demo.0 \
        "$SOURCE_CMD && sleep 16 && echo '=== DEMO OVERLAY ===' && ros2 run g1_detection overlay_node --ros-args -p color_topic:=/zed/zed_node/rgb/color/rect/image -p camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info -p world_frame:=odom -p base_frame:=pelvis -p record:=$([[ "$RECORD" == "1" ]] && echo true || echo false) -p record_path:=$DEMO_MP4" Enter
fi

# Focus core window
tmux select-window -t $SESSION:core

echo ""
echo "Attach with:    tmux attach -t $SESSION"
echo "Stop with:      ./run_zed_e2e.sh stop"
echo ""
echo "Send a goal (from any sourced shell):"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}\" --feedback"
echo ""
echo "Drive the real robot:  WITH_ROBOT=1 NET_IFACE=<nic> ./run_zed_e2e.sh   (default: perception-only)"
echo "ZED VIO for odometry:  USE_ZED_ODOM=1 ./run_zed_e2e.sh                 (default: RTAB-Map odom)"
echo ""
if [[ "$OVERLAY" == "1" ]]; then
    echo "Demo overlay:          window 'demo'  (tmux select-window -t $SESSION:demo)"
    echo "  watch live:          ros2 run rqt_image_view rqt_image_view /g1/overlay/image"
    if [[ "$RECORD" == "1" ]]; then
        echo "  recording to:        $DEMO_MP4"
        echo "  IMPORTANT:           stop with './run_zed_e2e.sh stop' so the MP4 is flushed."
    fi
    echo "  disable:             OVERLAY=0 ./run_zed_e2e.sh   (or RECORD=0 to view without saving)"
    echo ""
fi

# Auto-attach if interactive
if [[ -t 1 ]]; then
    sleep 1
    tmux attach -t $SESSION
fi
