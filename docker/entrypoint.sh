#!/bin/bash
# Sources ROS 2 + the g1_slam workspace (if built), then runs the given command.
set -e

source /opt/ros/humble/setup.bash

WS=/home/tracelab/Desktop/g1_slam/ros2_ws
if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
fi

# Keep DDS discovery on the host network sane.
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

exec "$@"
