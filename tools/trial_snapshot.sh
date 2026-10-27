#!/bin/bash
# Snapshot the running ROS graph into a trial folder: nodes, topics, every node's
# live parameters, the TF tree and the odom->pelvis transform. Started by
# run_zed_e2e.sh once the stack is up (~35 s). Takes ~30 s; prints "done".
#
#   bash tools/trial_snapshot.sh <trial_dir>
set -u
D="$1/snapshot"
mkdir -p "$D/params" "$D/tf"

ros2 node list        > "$D/nodes.txt"  2>&1
ros2 topic list -t    > "$D/topics.txt" 2>&1

# Live parameter values, which is what actually ran (YAML files can be overridden).
while read -r node; do
    [[ -z "$node" || "$node" != /* ]] && continue
    timeout 15 ros2 param dump "$node" > "$D/params/$(echo "${node#/}" | tr '/' '_').yaml" 2>&1
done < "$D/nodes.txt"

# TF tree (frames_*.pdf / .gv). Also answers "was there a map frame?".
( cd "$D/tf" && timeout 20 ros2 run tf2_tools view_frames > view_frames.log 2>&1 )
# odom->pelvis right now: z tells where the odom origin sits relative to the robot.
PYTHONUNBUFFERED=1 timeout 4 ros2 run tf2_ros tf2_echo odom pelvis > "$D/tf/odom_to_pelvis.txt" 2>&1

echo "Snapshot written to $D - done."
