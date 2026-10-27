#!/bin/bash
# Start / stop the ZED's native SVO recording for a trial, through the
# zed-ros2-wrapper services (v5.3.1: <node>/start_svo_rec, <node>/stop_svo_rec).
# An SVO holds the full stereo video, IMU and timestamps, and can be replayed
# through the ZED SDK to regenerate depth, tracking and point clouds offline.
#
#   bash tools/trial_svo.sh <trial_dir> start|stop
#
# Env: SVO_COMPRESSION, as mapped by wrapper v5.3.1:
#      1 = H264, 0 or 2 = H265 (default), 3 = H264 lossless, 4 = H265 lossless,
#      5 = lossless. The lossless modes are very large.
# Stop through this script ('./run_zed_e2e.sh stop' does) while the camera node
# is still running; killing the node first can leave the file unfinished.
set -u
TRIAL_DIR="$1"; ACTION="${2:-}"
NODE=/zed/zed_node
LOG="$TRIAL_DIR/svo.log"

case "$ACTION" in
    start)
        for _ in $(seq 1 60); do        # the camera node takes ~10 s to come up
            ros2 service list 2>/dev/null | grep -qx "$NODE/start_svo_rec" && break
            sleep 1
        done
        echo "[$(date --iso-8601=seconds)] start" >> "$LOG"
        timeout 15 ros2 service call "$NODE/start_svo_rec" zed_msgs/srv/StartSvoRec \
            "{svo_filename: '$TRIAL_DIR/camera.svo2', compression_mode: ${SVO_COMPRESSION:-2}, bitrate: 0, target_framerate: 0, input_transcode: false}" \
            2>&1 | tee -a "$LOG"
        if grep -q "success=True" "$LOG"; then
            echo "SVO recording -> $TRIAL_DIR/camera.svo2"
        else
            echo "WARNING: SVO recording did not start (see $LOG)"
        fi
        ;;
    stop)
        echo "[$(date --iso-8601=seconds)] stop" >> "$LOG"
        timeout 10 ros2 service call "$NODE/stop_svo_rec" std_srvs/srv/Trigger 2>&1 | tee -a "$LOG"
        ;;
    *)
        echo "usage: $0 <trial_dir> start|stop"; exit 2 ;;
esac
