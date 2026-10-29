#!/bin/bash
# Snapshot the running ROS graph into a trial folder: nodes, topics, every node's
# live parameters, lifecycle states, the TF tree and the odom->pelvis transform.
# Started by run_zed_e2e.sh once the stack is up (~35 s). Takes a few seconds
# (tools/graph_snapshot.py, one process); prints "done".
#
#   bash tools/trial_snapshot.sh <trial_dir>
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

if ! timeout 60 python3 "$HERE/graph_snapshot.py" "$1"; then
    echo "WARNING: graph snapshot failed or timed out; see above."
fi

echo "Snapshot written to $1/snapshot - done."
