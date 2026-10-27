#!/bin/bash
# Record a trial's rosbag into <trial_dir>/bag. Started by run_zed_e2e.sh
# (window 'trial'); stop it with SIGINT ('./run_zed_e2e.sh stop' does) so the
# bag's metadata.yaml is written.
#
#   bash tools/trial_record.sh <trial_dir>
#
# Env:
#   BAG_IMAGES=compressed|raw|none   camera images (default compressed)
#   BAG_CLOUD=0|1                    ZED point cloud, ~15 MB/frame (default 0)
#   BAG_EXTRA_TOPICS="/a /b"         e.g. motion-capture topics for ground truth
#
# Topics that never appear simply record nothing; tools/trial_check.py reports them.
set -u
TRIAL_DIR="$1"
BAG_IMAGES="${BAG_IMAGES:-compressed}"
BAG_CLOUD="${BAG_CLOUD:-0}"

TOPICS=(
    # Perception and the safety decision
    /g1/detections            # per-frame JSON: boxes, tracks, ranges, forecasts, cmd, timing
    /g1/human_cmd             # STOP / SLOW_DOWN / NORMAL_OPERATION as published
    /humans/markers
    /human_obstacle_cloud
    # What robot_node actually sent to the robot (after the override)
    /g1/robot_cmd
    # Trial annotations (tools/trial_event.py, tools/send_goal.py)
    /g1/trial/events
    /g1/trial/goal
    # Navigation
    /goal_pose
    /plan
    /local_plan
    /cmd_vel                  # Nav2 output, BEFORE the safety override
    /cmd_vel_nav              # controller output before the velocity smoother, if present
    /behavior_tree_log
    /navigate_to_pose/_action/status
    /navigate_to_pose/_action/feedback
    /local_costmap/costmap
    /global_costmap/costmap
    # State estimation
    /odom
    /zed/zed_node/odom
    /zed/zed_node/pose
    /zed/zed_node/imu/data
    /tf
    /tf_static
    # Camera
    /zed/zed_node/rgb/color/rect/camera_info
    /g1/overlay/image/compressed
    # System
    /rosout
    /diagnostics
    /parameter_events
    /robot_description
)

case "$BAG_IMAGES" in
    compressed) TOPICS+=(/zed/zed_node/rgb/color/rect/image/compressed) ;;
    raw)        TOPICS+=(/zed/zed_node/rgb/color/rect/image /zed/zed_node/depth/depth_registered) ;;
    none)       ;;
    *)          echo "BAG_IMAGES must be compressed|raw|none (got '$BAG_IMAGES')"; exit 2 ;;
esac
[[ "$BAG_CLOUD" == "1" ]] && TOPICS+=(/zed/zed_node/point_cloud/cloud_registered)
if [[ -n "${BAG_EXTRA_TOPICS:-}" ]]; then
    read -r -a EXTRA <<< "$BAG_EXTRA_TOPICS"
    TOPICS+=("${EXTRA[@]}")
fi

printf '%s\n' "${TOPICS[@]}" > "$TRIAL_DIR/bag_topics.txt"
echo "Recording ${#TOPICS[@]} topics -> $TRIAL_DIR/bag (list: bag_topics.txt)"
exec ros2 bag record -o "$TRIAL_DIR/bag" --include-hidden-topics "${TOPICS[@]}"
