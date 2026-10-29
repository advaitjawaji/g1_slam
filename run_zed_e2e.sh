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

# Trial recording (ON by default): every launch gets its own folder under
# TRIAL_ROOT holding the rosbag, every pane's console output, ROS node logs,
# parameter + TF snapshots, the robot's onboard state, the overlay MP4, the
# exact code that ran and software versions. See TRIALS.md. Off: RECORD_TRIAL=0.
#   TRIAL_NAME=corridor_2ped TRIAL_ENV=corridor TRIAL_PEDS=2 TRIAL_PED_SPEED=normal \
#     WITH_ROBOT=1 NET_IFACE=enp4s0 USE_ZED_ODOM=1 ./run_zed_e2e.sh
RECORD_TRIAL="${RECORD_TRIAL:-1}"
TRIAL_ROOT="${TRIAL_ROOT:-$HOME/g1_trials}"
TRIAL_NAME="${TRIAL_NAME:-}"; TRIAL_NAME="${TRIAL_NAME// /_}"
TRIAL_ENV="${TRIAL_ENV:-}"
TRIAL_PEDS="${TRIAL_PEDS:-}"
TRIAL_PED_SPEED="${TRIAL_PED_SPEED:-}"
TRIAL_NOTES="${TRIAL_NOTES:-}"
TRIAL_META="${TRIAL_META:-}"                 # pre-filled trial.yaml to copy instead
BAG_IMAGES="${BAG_IMAGES:-compressed}"       # compressed | raw | none
BAG_CLOUD="${BAG_CLOUD:-0}"                  # 1 = also bag the ZED point cloud (large)
BAG_EXTRA_TOPICS="${BAG_EXTRA_TOPICS:-}"     # e.g. motion-capture topics
SVO="${SVO:-1}"                              # 1 = ZED native SVO recording (stereo + IMU)
SVO_COMPRESSION="${SVO_COMPRESSION:-2}"      # wrapper v5.3.1: 1 H264, 2 H265, 3-5 lossless
CURRENT_TRIAL_FILE="$HOME/.g1_zed_current_trial"
LAST_TRIAL_FILE="$HOME/.g1_zed_last_trial"

# ── Helpers ───────────────────────────────────────────────────────────────
# Ported to ROS 2 Jazzy / aarch64 (Jetson Thor). CUDA path is arch-agnostic
# (/usr/local/cuda = JetPack symlink); was a hardcoded x86_64 path originally.
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
SOURCE_CMD="cd $REPO_DIR && source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && source $WS_DIR/install/setup.bash && export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH"

# Processes that only finalise their output on a clean shutdown: the MP4
# (overlay_node), the rosbag's metadata.yaml, and the robot-state / system CSVs.
# Patterns name the actual processes, not the pane-log files (logs/<name>.log)
# whose 'sh -c cat >> ...' mirrors would otherwise match too.
RECORDER_PATTERNS=("overlay_node" "ros2 bag record" "g1_state_logger.py" "sys_monitor.py")

# Stop the robot BEFORE anything slow (SVO close, recorder flush). SIGINT lets
# robot_node run its shutdown handler, which calls StopMove(); a SIGHUP from
# tmux kill-session would skip it. Nav2 can no longer drive the robot after this.
stop_robot() {
    local pat="robot_node --ros-args"        # the node itself, not files named robot_node*
    pgrep -f "$pat" >/dev/null 2>&1 || return 0
    echo "Stopping robot_node (StopMove)..."
    pkill -INT -f "$pat" || true
    for _ in $(seq 1 25); do                # up to 5 s
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
        sleep 0.2
    done
    echo "  robot_node did not exit; killing it."
    pkill -KILL -f "$pat" || true
}

# Close the ZED's SVO through the wrapper's service while the camera node is
# still running; killing the node first can leave the SVO unfinished.
stop_svo() {
    [[ -f "$CURRENT_TRIAL_FILE" ]] || return 0
    local dir
    dir="$(cat "$CURRENT_TRIAL_FILE")"
    [[ -f "$dir/svo.log" ]] || return 0                       # never started
    pgrep -f "zed_camera.launch.py" >/dev/null 2>&1 || return 0
    echo "Closing ZED SVO recording..."
    bash -c "$SOURCE_CMD && bash '$REPO_DIR/tools/trial_svo.sh' '$dir' stop" >/dev/null 2>&1 || true
}

# ORDER MATTERS. SIGINT the recorders and wait for them BEFORE tmux
# kill-session SIGHUPs every pane; killing the session first leaves an
# unplayable MP4 and a bag with no metadata.
flush_recorders() {
    local p any=0 alive
    for p in "${RECORDER_PATTERNS[@]}"; do
        if pgrep -f "$p" >/dev/null 2>&1; then pkill -INT -f "$p" || true; any=1; fi
    done
    [[ $any == 1 ]] || return 0
    echo "Flushing recordings (video, rosbag, robot state)..."
    for _ in $(seq 1 60); do                # up to 12 s
        alive=0
        for p in "${RECORDER_PATTERNS[@]}"; do pgrep -f "$p" >/dev/null 2>&1 && alive=1; done
        [[ $alive == 0 ]] && { echo "  done."; return 0; }
        sleep 0.2
    done
    echo "  a recorder did not exit; forcing (its file may be truncated)."
    for p in "${RECORDER_PATTERNS[@]}"; do pkill -KILL -f "$p" || true; done
}

# Mirror a pane's terminal output into the trial folder (no-op when not recording).
log_pane() {
    [[ "$RECORD_TRIAL" == "1" ]] || return 0
    tmux pipe-pane -o -t "$1" "cat >> '$TRIAL_DIR/logs/$2.log'"
}

# Close out the trial a previous launch was recording, then check it.
finish_trial() {
    [[ -f "$CURRENT_TRIAL_FILE" ]] || return 0
    local dir
    dir="$(cat "$CURRENT_TRIAL_FILE")"
    mv "$CURRENT_TRIAL_FILE" "$LAST_TRIAL_FILE"
    [[ -d "$dir" ]] || return 0
    echo "end_time: $(date --iso-8601=seconds)" >> "$dir/launch_info.yaml"
    echo ""
    echo "Trial saved: $dir"
    # Sourced, so the behaviour flags can read the bag with rosbag2_py.
    bash -c "$SOURCE_CMD && python3 '$REPO_DIR/tools/trial_check.py' '$dir'" || true
}

# ── stop ──────────────────────────────────────────────────────────────────
if [[ "${1:-}" == "stop" ]]; then
    stop_robot
    stop_svo
    flush_recorders

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
    pkill -f trial_snapshot || true
    echo "Stopped."
    finish_trial
    exit 0
fi

# ── Deps ──────────────────────────────────────────────────────────────────
command -v tmux >/dev/null || { echo "Installing tmux..."; sudo apt install -y tmux; }

# ZED-odom mode relays ZED VIO onto /rtabmap/odom via topic_tools relay.
if [[ "$USE_ZED_ODOM" == "1" ]]; then
    ( source /opt/ros/${ROS_DISTRO_NAME}/setup.bash 2>/dev/null; \
      ros2 pkg executables topic_tools 2>/dev/null | grep -q relay ) \
      || { echo "Installing ros-${ROS_DISTRO_NAME}-topic-tools (ZED-odom relay)..."; sudo apt install -y ros-${ROS_DISTRO_NAME}-topic-tools; }
fi

URDF_PATH="$(source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && source $WS_DIR/install/setup.bash 2>/dev/null && ros2 pkg prefix g1_description 2>/dev/null)/share/g1_description/urdf/g1_29dof.urdf"
NAV2_PARAMS="$(source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && source $WS_DIR/install/setup.bash 2>/dev/null && ros2 pkg prefix g1_bringup 2>/dev/null)/share/g1_bringup/config/nav2_params_hw.yaml"
# ZED wrapper parameter overrides (keeps VIO odometry continuous across loop
# closures; see the file). Installed with g1_bringup, so rebuild it after edits.
ZED_OVERRIDES="$(dirname "$NAV2_PARAMS")/zed_overrides.yaml"
if [[ -f "$ZED_OVERRIDES" ]]; then
    ZED_OVERRIDE_ARG="ros_params_override_path:=$ZED_OVERRIDES"
else
    echo "WARNING: $ZED_OVERRIDES not installed (colcon build --packages-select g1_bringup);"
    echo "         the ZED runs with wrapper defaults, so /odom can jump on loop closures."
    ZED_OVERRIDES=""; ZED_OVERRIDE_ARG=""
fi

# ── Kill old session ──────────────────────────────────────────────────────
# A previous launch that was never stopped may still be recording: close it out
# cleanly so its trial folder stays usable.
stop_robot
stop_svo
flush_recorders
tmux kill-session -t $SESSION 2>/dev/null || true
finish_trial

TRIAL_DIR=""
if [[ "$RECORD_TRIAL" == "1" ]]; then
    TRIAL_DIR="$TRIAL_ROOT/$(date +%Y%m%d_%H%M%S)${TRIAL_NAME:+_$TRIAL_NAME}"
fi

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
if [[ "$RECORD_TRIAL" == "1" ]]; then
    echo "  Trial:       RECORDING -> $TRIAL_DIR  (images: $BAG_IMAGES, cloud: $BAG_CLOUD, SVO: $SVO)"
else
    echo "  Trial:       not recording (RECORD_TRIAL=0)"
fi
echo "============================================================"
echo "Starting in 2 s (Ctrl+C to cancel)..."
sleep 2

# ── Trial folder ──────────────────────────────────────────────────────────
if [[ "$RECORD_TRIAL" == "1" ]]; then
    mkdir -p "$TRIAL_DIR/logs" "$TRIAL_DIR/ros_logs"
    echo "$TRIAL_DIR" > "$CURRENT_TRIAL_FILE"
    cat > "$TRIAL_DIR/launch_info.yaml" <<EOF
# Written by run_zed_e2e.sh at launch; end_time is appended by './run_zed_e2e.sh stop'.
start_time: $(date --iso-8601=seconds)
host: $(hostname)
launcher: run_zed_e2e.sh
with_robot: $WITH_ROBOT
net_iface: "$NET_IFACE"
use_zed_odom: $USE_ZED_ODOM
overlay: $OVERLAY
record_video: $RECORD
bag_images: $BAG_IMAGES
bag_cloud: $BAG_CLOUD
bag_extra_topics: "$BAG_EXTRA_TOPICS"
svo: $SVO
svo_compression: $SVO_COMPRESSION
git_commit: $(git -C "$REPO_DIR" rev-parse HEAD 2>/dev/null || echo unknown)
git_dirty: $([[ -n "$(git -C "$REPO_DIR" status --porcelain --untracked-files=no 2>/dev/null)" ]] && echo true || echo false)
nav2_params: "$NAV2_PARAMS"
zed_overrides: "$ZED_OVERRIDES"
urdf: "$URDF_PATH"
EOF
    TRIAL_NAME="$TRIAL_NAME" TRIAL_ENV="$TRIAL_ENV" TRIAL_PEDS="$TRIAL_PEDS" \
    TRIAL_PED_SPEED="$TRIAL_PED_SPEED" TRIAL_NOTES="$TRIAL_NOTES" TRIAL_META="$TRIAL_META" \
    ROS_DISTRO_NAME="$ROS_DISTRO_NAME" \
        bash "$REPO_DIR/tools/trial_init.sh" "$TRIAL_DIR" "$REPO_DIR" "$WS_DIR" "$NAV2_PARAMS" \
        || echo "WARNING: trial_init.sh failed; continuing without code/version snapshot."
    # Every ROS node writes its log file into the trial folder too.
    SOURCE_CMD="$SOURCE_CMD && export ROS_LOG_DIR='$TRIAL_DIR/ros_logs'"
fi

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
log_pane $SESSION:core.0 zed
log_pane $SESSION:core.1 odom_source
log_pane $SESSION:core.2 detection
log_pane $SESSION:core.3 nav2

# Pane 0 - ZED
tmux send-keys -t $SESSION:core.0 \
    "$SOURCE_CMD && echo '=== ZED ===' && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i $ZED_TRACK_ARGS $ZED_OVERRIDE_ARG" Enter

# Pane 1 - odom source: RTAB-Map VO (default) OR a relay of ZED VIO odom
if [[ "$USE_ZED_ODOM" == "1" ]]; then
    # ZED VIO owns odom->zed_camera_link (->pelvis via the static TF below).
    # Relay ZED odom onto /odom — the topic Nav2 reads (nav2_params_hw
    # odom_topic: /odom). Needs ros-${ROS_DISTRO_NAME}-topic-tools.
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

# Pane 3 - Nav2. navigation_launch.py's lifecycle manager brings every node up
# itself (autostart); a second manual STARTUP call only failed against the
# already-active nodes ("Aborting bringup") in the 2026-10-27 trials.
tmux send-keys -t $SESSION:core.3 \
    "$SOURCE_CMD && sleep 14 && echo '=== NAV2 (autostart) ===' && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false autostart:=true params_file:=$NAV2_PARAMS" Enter

# ── Window 'extras' ───────────────────────────────────────────────────────
tmux new-window -t $SESSION -n extras
tmux split-window -h -t $SESSION:extras
tmux split-window -v -t $SESSION:extras.0
tmux split-window -v -t $SESSION:extras.2
log_pane $SESSION:extras.0 static_tf
log_pane $SESSION:extras.1 robot_state_publisher
log_pane $SESSION:extras.2 rviz
log_pane $SESSION:extras.3 monitor

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
    "$SOURCE_CMD && sleep 16 && echo '=== RVIZ ===' && rviz2 -d /opt/ros/${ROS_DISTRO_NAME}/share/nav2_bringup/rviz/nav2_default_view.rviz" Enter

# Pane 3 - monitor
tmux send-keys -t $SESSION:extras.3 \
    "$SOURCE_CMD && sleep 18 && echo '=== MONITOR ===' && echo 'Commands:' && echo '  ros2 topic hz /cmd_vel' && echo '  ros2 topic echo /g1/human_cmd' && echo '  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...' && bash" Enter

# ── Window 'ctrl' : human-obstacle injector (+ optional robot driver) ──────
tmux new-window -t $SESSION -n ctrl
log_pane $SESSION:ctrl.0 human_obstacle

# Pane 0 - human_obstacle_node : /humans/markers -> /human_obstacle_cloud
#          (perception only, always on; costmap source is still opt-in in YAML)
tmux send-keys -t $SESSION:ctrl.0 \
    "$SOURCE_CMD && sleep 14 && echo '=== HUMAN OBSTACLE (PointCloud2) ===' && ros2 run g1_detection human_obstacle_node" Enter

if [[ "$WITH_ROBOT" == "1" ]]; then
    tmux split-window -v -t $SESSION:ctrl.0
    log_pane $SESSION:ctrl.1 robot_node
    # Pane 1 - robot_node : drives the real G1 from /cmd_vel.
    tmux send-keys -t $SESSION:ctrl.1 \
        "$SOURCE_CMD && sleep 8 && echo '=== ROBOT NODE — DRIVES THE G1 (robot must be STANDING, e-stop ready) ===' && ros2 run g1_robot robot_node --ros-args -p net_iface:=$NET_IFACE -r /cmd_vel:=/cmd_vel_smoothed" Enter
fi

# ── Window 'demo' : the annotated egocentric video ────────────────────────
if [[ "$OVERLAY" == "1" ]]; then
    if [[ "$RECORD_TRIAL" == "1" ]]; then
        DEMO_MP4="$TRIAL_DIR/overlay.mp4"      # the video belongs with its trial
    else
        mkdir -p "$DEMO_DIR"
        DEMO_MP4="$DEMO_DIR/g1_demo_$(date +%Y%m%d_%H%M%S).mp4"
    fi
    tmux new-window -t $SESSION -n demo
    log_pane $SESSION:demo.0 overlay
    # Starts after detection so intrinsics + the first /g1/detections have
    # landed; it renders regardless, so a late detector just means the opening
    # seconds have no boxes.
    tmux send-keys -t $SESSION:demo.0 \
        "$SOURCE_CMD && sleep 16 && echo '=== DEMO OVERLAY ===' && ros2 run g1_detection overlay_node --ros-args -p color_topic:=/zed/zed_node/rgb/color/rect/image -p camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info -p world_frame:=odom -p base_frame:=pelvis -p record:=$([[ "$RECORD" == "1" ]] && echo true || echo false) -p record_path:=$DEMO_MP4" Enter
fi

# ── Window 'trial' : everything that records the trial ────────────────────
# trial_pane <log-name> <command>: one pane per recorder, tiled so none is
# squeezed out as the count grows.
TRIAL_PANES=0
trial_pane() {
    local idx=$TRIAL_PANES
    if [[ $idx -gt 0 ]]; then
        tmux split-window -v -t $SESSION:trial.$((idx - 1))
        tmux select-layout -t $SESSION:trial tiled >/dev/null
    fi
    log_pane $SESSION:trial.$idx "$1"
    tmux send-keys -t $SESSION:trial.$idx "$2" Enter
    TRIAL_PANES=$((idx + 1))
}

if [[ "$RECORD_TRIAL" == "1" ]]; then
    tmux new-window -t $SESSION -n trial
    # rosbag. Starts early so it sees every topic as it appears.
    trial_pane bag_record \
        "$SOURCE_CMD && sleep 3 && echo '=== TRIAL ROSBAG ===' && BAG_IMAGES=$BAG_IMAGES BAG_CLOUD=$BAG_CLOUD BAG_EXTRA_TOPICS='$BAG_EXTRA_TOPICS' bash $REPO_DIR/tools/trial_record.sh '$TRIAL_DIR'"
    # Computer load (CPU / GPU / per-component), for the whole run.
    trial_pane sys_monitor \
        "$SOURCE_CMD && sleep 2 && echo '=== SYSTEM MONITOR ===' && python3 $REPO_DIR/tools/sys_monitor.py --out '$TRIAL_DIR/system.csv'"
    # One-off snapshot of nodes, live params and TF once the stack is up.
    trial_pane snapshot \
        "$SOURCE_CMD && sleep 35 && echo '=== TRIAL SNAPSHOT ===' && bash $REPO_DIR/tools/trial_snapshot.sh '$TRIAL_DIR'"
    if [[ "$SVO" == "1" ]]; then
        # ZED native recording; waits for the camera's service to appear.
        trial_pane svo \
            "$SOURCE_CMD && sleep 12 && echo '=== ZED SVO RECORDING ===' && SVO_COMPRESSION=$SVO_COMPRESSION bash $REPO_DIR/tools/trial_svo.sh '$TRIAL_DIR' start"
    fi
    if [[ "$WITH_ROBOT" == "1" ]]; then
        # G1 onboard state (rt/lowstate, torso IMU, ...). Subscribe-only.
        trial_pane robot_state \
            "$SOURCE_CMD && sleep 8 && echo '=== G1 STATE LOGGER (read-only) ===' && python3 $REPO_DIR/tools/g1_state_logger.py --out '$TRIAL_DIR/robot_state' --iface '$NET_IFACE'"
    fi
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
if [[ "$RECORD_TRIAL" == "1" ]]; then
    echo "Trial folder:          $TRIAL_DIR"
    echo "  wait for 'Snapshot ... done' in window 'trial' (~40 s) before starting the run"
    echo "  send a logged goal:  python3 tools/send_goal.py 3.0 0.0"
    echo "  mark a moment:       python3 tools/trial_event.py --kind ped_start \"P1 starts walking\""
    echo "  IMPORTANT:           stop with './run_zed_e2e.sh stop' so the bag and video close cleanly"
    echo ""
fi

# Auto-attach if interactive
if [[ -t 1 ]]; then
    sleep 1
    tmux attach -t $SESSION
fi
