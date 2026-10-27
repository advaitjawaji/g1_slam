#!/bin/bash
# Build the g1_slam ROS 2 Humble image for this machine.
#
#   ./docker/build.sh                 # ROS core + ML (torch/ultralytics), no Genesis
#   INSTALL_GENESIS=true ./docker/build.sh   # also attempt genesis-world (sim)
#   INSTALL_ML=false     ./docker/build.sh   # ROS only, no torch (smaller/faster)
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE="${IMAGE:-g1_slam:humble}"

# When invoked via `sudo` (needed if you're not in the docker group), id -u is 0.
# Fall back to the real invoking user so the container user matches the host.
HOST_UID="${SUDO_UID:-$(id -u)}"
HOST_GID="${SUDO_GID:-$(id -g)}"
[ "$HOST_UID" = "0" ] && HOST_UID=1000
[ "$HOST_GID" = "0" ] && HOST_GID=1000
echo "Building $IMAGE for UID:GID = $HOST_UID:$HOST_GID"

# The NVIDIA runtime injects the host's `render` group for /dev/dri access;
# creating it in the image keeps `groups` from warning about an unnamed GID.
RENDER_GID="$(getent group render | cut -d: -f3)"
RENDER_GID="${RENDER_GID:-993}"

docker build \
    --build-arg UID="$HOST_UID" \
    --build-arg GID="$HOST_GID" \
    --build-arg RENDER_GID="$RENDER_GID" \
    --build-arg INSTALL_ML="${INSTALL_ML:-true}" \
    --build-arg INSTALL_GENESIS="${INSTALL_GENESIS:-false}" \
    -t "$IMAGE" \
    "$DIR"

echo ""
echo "Built $IMAGE"
echo "Next:  ./docker/run.sh        (starts a shell in the container)"
echo "Then:  cd ros2_ws && colcon build --symlink-install   (first time only)"
