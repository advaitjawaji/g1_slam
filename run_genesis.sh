#!/bin/bash
# Single-command Genesis simulation launcher
# Opens a tmux session with all components in split panes
#
# Usage:
#   ./run_genesis.sh                        # kinematic mode
#   ./run_genesis.sh --policy path/to.pt   # RL walking mode

set -e

POLICY_ARG=""
if [ "$1" == "--policy" ] && [ -n "$2" ]; then
    POLICY_ARG="--policy $2"
fi

WS_DIR="$(cd "$(dirname "$0")/ros2_ws" && pwd)"
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── Check dependencies ────────────────────────────────────────────────────
if ! command -v tmux &> /dev/null; then
    echo "Installing tmux..."
    sudo apt install -y tmux
fi

if ! python3 -c "import genesis" &> /dev/null; then
    echo "ERROR: Genesis not installed. Run: pip3 install genesis-world"
    exit 1
fi

# Source helper
SOURCE_CMD="source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash"

# Kill any existing session
tmux kill-session -t g1_sim 2>/dev/null || true

echo ""
echo "============================================================"
echo "  G1 Genesis Simulation"
echo "  Repo: $REPO_DIR"
echo "  Mode: $([ -n "$POLICY_ARG" ] && echo 'RL walking' || echo 'kinematic floating')"
echo "============================================================"
echo ""
echo "Starting in 2 seconds... (Ctrl+C to cancel)"
sleep 2

# ── Create tmux session ───────────────────────────────────────────────────
tmux new-session -d -s g1_sim -x 220 -y 50

# Name the first window
tmux rename-window -t g1_sim:0 "Genesis Sim"

# ── Pane layout ───────────────────────────────────────────────────────────
# Split into 4 panes:
#   ┌─────────────────┬─────────────────┐
#   │  1. Genesis     │  2. ROS2 stack  │
#   ├─────────────────┼─────────────────┤
#   │  3. RViz log    │  4. Teleop      │
#   └─────────────────┴─────────────────┘

tmux split-window -h -t g1_sim:0
tmux split-window -v -t g1_sim:0.0
tmux split-window -v -t g1_sim:0.1

# ── Pane 0 (top-left): Genesis bridge ────────────────────────────────────
tmux send-keys -t g1_sim:0.0 \
    "cd $REPO_DIR && $SOURCE_CMD && echo '=== GENESIS BRIDGE ===' && python3 genesis_sim/genesis_node.py $POLICY_ARG" \
    Enter

# ── Pane 1 (top-right): ROS2 stack ───────────────────────────────────────
# Wait 8s for Genesis to build the scene before launching ROS2
tmux send-keys -t g1_sim:0.1 \
    "sleep 8 && $SOURCE_CMD && echo '=== ROS2 STACK ===' && ros2 launch g1_bringup genesis.launch.py rviz:=false" \
    Enter

# ── Pane 2 (bottom-left): RViz ───────────────────────────────────────────
# Wait 15s for SLAM to initialise
tmux send-keys -t g1_sim:0.2 \
    "sleep 15 && $SOURCE_CMD && echo '=== RVIZ ===' && ros2 run rviz2 rviz2" \
    Enter

# ── Pane 3 (bottom-right): Teleop ────────────────────────────────────────
tmux send-keys -t g1_sim:0.3 \
    "sleep 10 && source /opt/ros/humble/setup.bash && echo '=== TELEOP ===' && echo 'Use i/j/l/, to drive. k to stop.' && ros2 run teleop_twist_keyboard teleop_twist_keyboard" \
    Enter

# ── Attach to session ─────────────────────────────────────────────────────
echo ""
echo "Launching tmux session. Controls:"
echo "  Ctrl+B then arrow keys  — switch between panes"
echo "  Ctrl+B then d           — detach (keeps running)"
echo "  tmux attach -t g1_sim   — reattach later"
echo "  ./stop_genesis.sh       — stop everything"
echo ""
echo "RViz setup (after map appears ~15s):"
echo "  Fixed Frame → map"
echo "  Add → Map → /map"
echo "  Add → MarkerArray → /humans/markers"
echo "  Add → RobotModel → /robot_description"
echo ""

tmux attach-session -t g1_sim
