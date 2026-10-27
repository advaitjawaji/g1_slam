#!/bin/bash
# Record, at launch, everything about a trial that does not need ROS running:
# operator metadata, the exact code that ran, whether the installed build matches
# the source, and software versions. Called by run_zed_e2e.sh.
#
#   bash tools/trial_init.sh <trial_dir> <repo_dir> <ws_dir> <nav2_params>
#
# Metadata env (all optional): TRIAL_NAME TRIAL_ENV TRIAL_PEDS TRIAL_PED_SPEED
# TRIAL_NOTES, or TRIAL_META=<yaml> to copy a pre-filled file instead.
set -u
TRIAL_DIR="$1"; REPO_DIR="$2"; WS_DIR="$3"; NAV2_PARAMS="$4"
mkdir -p "$TRIAL_DIR/code" "$TRIAL_DIR/snapshot"

# ── Operator metadata ─────────────────────────────────────────────────────
if [[ -n "${TRIAL_META:-}" && -f "${TRIAL_META:-}" ]]; then
    cp "$TRIAL_META" "$TRIAL_DIR/trial.yaml"
else
    cat > "$TRIAL_DIR/trial.yaml" <<EOF
# Trial metadata. The launcher fills what it was given; complete the rest
# before or right after the trial. Analysis groups and filters on these fields.
trial_name: "${TRIAL_NAME:-}"
environment: "${TRIAL_ENV:-}"            # e.g. lab_open, corridor, atrium
n_pedestrians: "${TRIAL_PEDS:-}"
pedestrian_speed: "${TRIAL_PED_SPEED:-}" # instructed: slow / normal / brisk, or m/s
pedestrian_paths: ""                     # e.g. head-on, crossing at 90 deg, overtaking
robot_goal: ""                           # tools/send_goal.py also logs goals as events
ground_truth: ""                         # mocap / floor marks + video / none
lighting_surface: ""
operator: ""
g1_firmware: ""
outcome: ""                              # success / failure, with reason
notes: "${TRIAL_NOTES:-}"
EOF
fi

# ── The exact code that ran ───────────────────────────────────────────────
git -C "$REPO_DIR" rev-parse HEAD            > "$TRIAL_DIR/code/git_commit.txt" 2>&1
git -C "$REPO_DIR" status --porcelain        > "$TRIAL_DIR/code/git_status.txt" 2>&1
git -C "$REPO_DIR" diff HEAD                 > "$TRIAL_DIR/code/git_diff.patch" 2>&1
( cd "$REPO_DIR" && tar czf "$TRIAL_DIR/code/source_snapshot.tar.gz" \
      run_zed_e2e.sh tools \
      ros2_ws/src/g1_detection/g1_detection/*.py \
      ros2_ws/src/g1_robot/g1_robot/*.py \
      ros2_ws/src/g1_bringup/config 2>/dev/null )
cp "$NAV2_PARAMS" "$TRIAL_DIR/snapshot/nav2_params_hw.installed.yaml" 2>/dev/null

# The installed package is what runs, not src/. A stale build (src edited, not
# rebuilt) is flagged here so a trial is never attributed to code that didn't run.
{
    for pkg in g1_detection g1_robot; do
        for f in "$WS_DIR/src/$pkg/$pkg/"*.py; do
            name="$(basename "$f")"
            inst="$(find "$WS_DIR/install/$pkg" -path "*/$pkg/$name" 2>/dev/null | head -n 1)"
            if [[ -z "$inst" ]]; then
                echo "MISSING  $pkg/$name (not installed)"
            elif cmp -s "$f" "$inst"; then
                echo "same     $pkg/$name"
            else
                echo "STALE    $pkg/$name (installed copy differs from src - rebuild with colcon)"
            fi
        done
    done
    src_p="$WS_DIR/src/g1_bringup/config/nav2_params_hw.yaml"
    if cmp -s "$src_p" "$NAV2_PARAMS"; then echo "same     g1_bringup/nav2_params_hw.yaml"
    else echo "STALE    g1_bringup/nav2_params_hw.yaml (installed copy differs from src)"; fi
} > "$TRIAL_DIR/code/install_vs_src.txt"

# ── Versions ──────────────────────────────────────────────────────────────
{
    echo "== system";       uname -a; lsb_release -ds 2>/dev/null
    echo "== gpu";          nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1
    echo "== zed sdk";      grep -E "#define ZED_SDK_(MAJOR|MINOR|SUB|PATCH)_VERSION" \
                                /usr/local/zed/include/sl/Camera.hpp 2>&1
    echo "== zed wrapper";  git -C "$WS_DIR/src/zed-ros2-wrapper" describe --tags --always 2>&1
    echo "== ros packages"; dpkg-query -W -f='${Package} ${Version}\n' \
                                'ros-humble-nav2*' 'ros-humble-rosbag2*' 'ros-humble-rclpy' \
                                'ros-humble-topic-tools' 'ros-humble-tf2-ros' 2>&1
    echo "== python";       python3 --version 2>&1
    python3 -m pip show ultralytics torch numpy opencv-python unitree_sdk2py 2>/dev/null \
        | grep -E '^(Name|Version):'
    echo "== yolo weights"; sha256sum "$REPO_DIR/yolo26n.pt" 2>&1
} > "$TRIAL_DIR/code/versions.txt"

echo "Trial metadata, code snapshot and versions written to $TRIAL_DIR"
grep -E "^(STALE|MISSING)" "$TRIAL_DIR/code/install_vs_src.txt" \
    && echo "WARNING: installed build differs from src (see code/install_vs_src.txt)"
exit 0
