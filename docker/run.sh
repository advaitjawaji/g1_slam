#!/bin/bash
# Launch the g1_slam container on this Jetson Thor host.
#
# Design choices:
#   --network/--ipc host : ROS 2 DDS discovery + Unitree SDK reach the LAN/robot.
#   -v $REPO:$REPO       : repo bind-mounted at the SAME path, so run_genesis.sh,
#                          activate_nav2.sh, demo_tour.sh etc. work unchanged.
#   nvidia runtime       : auto-enabled if available (GPU for YOLO/ZED/Genesis).
#   --privileged + /dev  : USB cameras (RealSense/ZED) and serial devices.
#   X11                  : rviz2 / GUI windows forward to the host display.
#
# Usage:
#   ./docker/run.sh                       # interactive bash shell
#   ./docker/run.sh ./run_genesis.sh      # run a specific command
set -e

IMAGE="${IMAGE:-g1_slam:humble}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
NAME="g1_slam"

# Allow the container to draw on the host X server.
xhost +local:root >/dev/null 2>&1 || true

# Enable the NVIDIA runtime only if Docker actually has it (it does on this host).
NV=()
if docker info 2>/dev/null | grep -qiE 'Runtimes:.*nvidia|nvidia'; then
    NV=(--runtime nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all)
fi

# Reuse a running container if present, else start a fresh one.
if [ "$(docker ps -q -f name=^/${NAME}$)" ]; then
    exec docker exec -it "$NAME" /entrypoint.sh "${@:-bash}"
fi

exec docker run -it --rm \
    --name "$NAME" \
    --network host \
    --ipc host \
    --privileged \
    "${NV[@]}" \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v "$REPO":"$REPO" \
    "$IMAGE" "${@:-bash}"
