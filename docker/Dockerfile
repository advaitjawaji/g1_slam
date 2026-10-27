# g1_slam — ROS 2 Humble dev container
#
# Why a container: this stack targets ROS 2 Humble, which has NO binary packages
# for Ubuntu 24.04 (noble). This host is a Jetson AGX Thor (aarch64, JetPack 7).
# We base off the official multi-arch `ros:humble` (ros-base, guaranteed arm64)
# and install the `ros-humble-desktop` metapackage + rviz2 via apt, exactly like
# the original install_ros2.sh. (The `-desktop` image tags live under the osrf/ros
# namespace and are not reliably built for arm64, so we avoid them.)
#
# The repo itself is NOT copied in — it is bind-mounted at runtime at the SAME
# host path so every top-level script (run_genesis.sh, activate_nav2.sh, ...)
# works unchanged inside the container.

FROM ros:humble

# Match the host user so files created on the mounted volume (colcon build/,
# install/, log/) stay owned by you, not root.
ARG UID=1000
ARG GID=1000

# Heavy / arm64-flaky Python deps are opt-in:
#   INSTALL_ML=true       -> torch + ultralytics (needed for g1_detection / YOLO)
#   INSTALL_GENESIS=true  -> genesis-world       (needed for the Genesis sim path)
ARG INSTALL_ML=true
ARG INSTALL_GENESIS=false

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ── ROS + system deps (mirrors install_ros2.sh). ros-humble-desktop brings
#    rviz2 + the GUI/demo tooling on top of the ros-base image. ──
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-humble-rtabmap-ros \
        ros-humble-realsense2-camera \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
        ros-humble-cv-bridge \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-teleop-twist-keyboard \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-argcomplete \
        python3-pip \
        tmux git nano wget usbutils sudo x11-apps \
    && rm -rf /var/lib/apt/lists/*

# ── Core Python deps (from requirements.txt, minus pyrealsense2 which is never
#    imported by the code and has no aarch64 wheel). These fix the numpy/colcon
#    gotchas noted in STATE.md §8. ──
RUN pip3 install --no-cache-dir -U packaging setuptools "coverage>=7.0" \
    && pip3 install --no-cache-dir \
        numpy opencv-python matplotlib pyzmq requests lap

# ── ML deps: torch + ultralytics (YOLO). CPU aarch64 wheels from PyPI.
#    For GPU acceleration on this Jetson, replace torch with the JetPack wheel
#    (see docker/README.md). ──
RUN if [ "$INSTALL_ML" = "true" ]; then \
        pip3 install --no-cache-dir torch ultralytics ; \
    else echo "skip ML deps (INSTALL_ML=$INSTALL_ML)"; fi

# ── Genesis simulator (best-effort; taichi/genesis arm64 support is fragile).
#    A failure here does NOT break the image — install manually if needed. ──
RUN if [ "$INSTALL_GENESIS" = "true" ]; then \
        pip3 install --no-cache-dir genesis-world \
        || echo "WARN: genesis-world failed to install on aarch64 — see docker/README.md"; \
    else echo "skip Genesis (INSTALL_GENESIS=$INSTALL_GENESIS)"; fi

# Host's `render` GID. The NVIDIA runtime injects this group into the container
# for /dev/dri access; creating it here just stops `groups` warning about an
# unnamed GID. Cosmetic only. Declared here (not at the top) so that changing it
# does not invalidate the cache of the expensive apt/pip layers above.
ARG RENDER_GID=993

# ── Create a user matching the host UID/GID (idempotent: reuse if it exists) ──
RUN set -eux; \
    if ! getent group ${GID} >/dev/null; then groupadd -g ${GID} tracelab; fi; \
    if ! id -u ${UID} >/dev/null 2>&1; then \
        useradd -m -u ${UID} -g ${GID} -s /bin/bash tracelab; \
    fi; \
    if ! getent group ${RENDER_GID} >/dev/null; then groupadd -g ${RENDER_GID} render; fi; \
    USER_NAME="$(id -un ${UID})"; \
    HOME_DIR="$(getent passwd ${UID} | cut -d: -f6)"; \
    mkdir -p "$HOME_DIR"; \
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/g1slam; \
    usermod -aG video,dialout,plugdev "$USER_NAME" || true; \
    usermod -aG "$(getent group ${RENDER_GID} | cut -d: -f1)" "$USER_NAME" || true; \
    printf '%s\n' 'export PS1="\[\e[1;36m\](g1-docker)\[\e[0m\] \[\e[1;32m\]\u\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ "' >> "$HOME_DIR/.bashrc"; \
    echo "source /opt/ros/humble/setup.bash" >> "$HOME_DIR/.bashrc"; \
    echo "[ -f /home/tracelab/Desktop/g1_slam/ros2_ws/install/setup.bash ] && source /home/tracelab/Desktop/g1_slam/ros2_ws/install/setup.bash" >> "$HOME_DIR/.bashrc"; \
    chown -R ${UID}:${GID} "$HOME_DIR"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER tracelab
WORKDIR /home/tracelab/Desktop/g1_slam
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
