#!/usr/bin/env bash
# Pre-flight for the demo overlay. Run it AFTER ./run_zed_e2e.sh has settled
# (~25 s) and BEFORE you start recording the take that matters.
#
#   ./tools/preflight_overlay.sh
#
# Checks each thing the video depends on, in the order it can fail, and says
# which of the two headline features you are actually going to get.

WORLD_FRAME="${WORLD_FRAME:-odom}"
COLOR_TOPIC="${COLOR_TOPIC:-/zed/zed_node/rgb/color/rect/image}"
INFO_TOPIC="${INFO_TOPIC:-/zed/zed_node/rgb/color/rect/camera_info}"

pass=0; warn=0; fail=0
ok()   { echo "  [ OK ]  $1"; pass=$((pass+1)); }
no()   { echo "  [FAIL]  $1"; fail=$((fail+1)); }
meh()  { echo "  [WARN]  $1"; warn=$((warn+1)); }

has_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
hz() { timeout "${2:-5}" ros2 topic hz "$1" 2>/dev/null | grep -m1 -oP 'average rate: \K[0-9.]+'; }

echo ""
echo "=== 1. Camera ==="
has_topic "$COLOR_TOPIC" && ok "color topic $COLOR_TOPIC" || no "no $COLOR_TOPIC — is the ZED up?"
has_topic "$INFO_TOPIC"  && ok "camera_info $INFO_TOPIC"  || no "no $INFO_TOPIC — overlay cannot start without intrinsics"

OPTICAL=$(timeout 5 ros2 topic echo "$INFO_TOPIC" --once 2>/dev/null \
          | grep -m1 'frame_id:' | awk '{print $2}' | tr -d '"\r')
[[ -n "$OPTICAL" ]] && ok "optical frame = $OPTICAL" \
                    || meh "could not read camera_info frame_id"

echo ""
echo "=== 2. TF chain (drives the AR layers) ==="
if [[ -n "$OPTICAL" ]]; then
    if timeout 6 ros2 run tf2_ros tf2_echo "$OPTICAL" "$WORLD_FRAME" 2>&1 \
         | grep -q "Translation"; then
        ok "$WORLD_FRAME -> $OPTICAL resolves"
    else
        no "$WORLD_FRAME -> $OPTICAL does NOT resolve"
        echo "         => no human forecast lines, no planned/walked path, no BEV tracks."
        echo "         => bounding boxes will still work."
        echo "         Debug: ros2 run tf2_tools view_frames"
    fi
fi
if timeout 6 ros2 run tf2_ros tf2_echo "$WORLD_FRAME" pelvis 2>&1 | grep -q "Translation"; then
    ok "$WORLD_FRAME -> pelvis resolves (true trajectory + floor height)"
else
    no "$WORLD_FRAME -> pelvis missing — walked path and ground plane will be wrong"
fi

echo ""
echo "=== 3. Detection ==="
if has_topic /g1/detections; then
    ok "/g1/detections present"
    PAY=$(timeout 6 ros2 topic echo /g1/detections --once 2>/dev/null)
    NDET=$(grep -o '"id"' <<<"$PAY" | wc -l)
    if [[ "$NDET" -gt 0 ]]; then
        ok "$NDET human(s) detected right now -> BOUNDING BOXES WILL RENDER"
    else
        meh "no humans in view right now (stand in front of the camera and re-run)"
    fi
    if grep -q '"world_frame":null' <<<"$PAY"; then
        no "payload world_frame is null -> forecasts DISABLED (TF problem above)"
    else
        ok "payload carries world frame"
    fi
    NPRED=$(grep -o '"pred":\[\[' <<<"$PAY" | wc -l)
    if [[ "$NPRED" -gt 0 ]]; then
        ok "$NPRED track(s) have a forecast -> PREDICTED TRAJECTORY WILL RENDER"
    else
        meh "no forecasts yet. Human_dtp only forecasts a person who is MOVING"
        echo "         (>0.08 m of travel, 3+ confirmed frames, within 6 m)."
        echo "         Have someone WALK ACROSS the view and re-run this check."
    fi
else
    no "/g1/detections missing — detection_node is not running or not rebuilt"
    echo "         colcon build --packages-select g1_detection && source install/setup.bash"
fi

echo ""
echo "=== 4. Nav2 plan (cyan path) ==="
has_topic /plan && ok "/plan present" \
    || meh "/plan absent — send a goal; without one there is no planned path to draw"

echo ""
echo "=== 5. Overlay output ==="
if has_topic /g1/overlay/image; then
    ok "/g1/overlay/image present"
    RATE=$(hz /g1/overlay/image 6)
    [[ -n "$RATE" ]] && ok "publishing at ${RATE} Hz" || no "topic exists but nothing publishing"
else
    no "/g1/overlay/image missing — overlay_node not running"
fi

echo ""
echo "-------------------------------------------------------------"
echo "  $pass ok, $warn warning, $fail fail"
[[ $fail -eq 0 ]] && echo "  Good to record." \
                  || echo "  Fix the FAILs above before the take."
echo "  Remember: stop with './run_zed_e2e.sh stop' or the MP4 won't finalise."
echo "-------------------------------------------------------------"
echo ""
