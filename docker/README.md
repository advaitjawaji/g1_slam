# Running g1_slam in Docker (Ubuntu 24.04 / Jetson Thor)

This stack is built for **ROS 2 Humble**, which has **no binary packages for
Ubuntu 24.04 (noble)** — Humble targets 22.04 (jammy). This host is a **Jetson
AGX Thor** (aarch64, JetPack 7, CUDA 13.2, Ubuntu 24.04), so instead of porting
the whole stack to ROS 2 Jazzy we run Humble inside a container.

The official multi-arch `ros:humble` (ros-base) image has an **arm64** build that
runs on the 24.04 host kernel; we add the `ros-humble-desktop` metapackage
(rviz2 + GUI tooling) via apt on top of it. (The `-desktop` *image* tags live
under the `osrf/ros` namespace and aren't reliably built for arm64, so we avoid
them.)

## Prerequisite: docker access

This user is not in the `docker` group yet, so either prefix commands with
`sudo`, or add yourself once and re-login:

```bash
sudo usermod -aG docker $USER   # then log out / back in (or: newgrp docker)
```

## TL;DR

```bash
./docker/build.sh                                  # build the image (~10–20 min first time)
./docker/run.sh                                    # drop into a shell in the container
# --- inside the container, first time only: ---
cd ros2_ws && colcon build --symlink-install && cd ..
source ros2_ws/install/setup.bash
```

The repo is bind-mounted at the **same path** (`/home/tracelab/Desktop/g1_slam`),
so every top-level script works inside the container exactly as documented in
`README.md` / `STATE.md`.

## What runs, and what still needs work

| Path | Status in container |
|---|---|
| ROS 2 Humble, rviz2, Nav2, RTAB-Map, colcon build | ✅ works out of the box |
| Genesis sim (validated demo) | ⚠️ needs `genesis-world` — see below |
| YOLO detection (`g1_detection`) | ⚠️ CPU torch by default; GPU needs a JetPack torch wheel |
| ZED 2 camera (`zed_wrapper`) | ❌ not baked in — see "ZED / CUDA" |
| Unitree G1 hardware (`g1_robot`) | ⚠️ needs `unitree_sdk2py` (`pip install unitree_sdk2py`) |

## Build options

`build.sh` passes your host UID/GID so files created on the mounted volume
(`build/`, `install/`, `log/`) stay owned by you. Toggles via env vars:

```bash
INSTALL_GENESIS=true ./docker/build.sh   # attempt genesis-world (the sim path)
INSTALL_ML=false     ./docker/build.sh   # ROS only, skip torch/ultralytics (smaller image)
```

`torch`, `ultralytics`, and `genesis-world` are the pieces most likely to be
slow or flaky to install on aarch64, which is why they are opt-in layers rather
than baked into the base.

## GPU / CUDA on this Jetson

`run.sh` auto-enables the NVIDIA container runtime (already configured on this
host in `/etc/docker/daemon.json`), so `nvidia-smi` works inside the container.

**But**: the PyPI `torch` wheel installed by default is **CPU-only** on aarch64.
For real GPU-accelerated YOLO / policy inference you must replace it with the
JetPack/L4T torch build that matches this device's CUDA (13.2):

```bash
# inside the container — install NVIDIA's Jetson torch wheel (check the exact
# URL for your JetPack version at developer.download.nvidia.com/compute/redist/jp)
pip3 uninstall -y torch
pip3 install --no-cache-dir <jetson-torch-wheel-url>
```

## ZED 2 camera / CUDA

The ZED path is **not** baked into this image. STATE.md §8 and ZED2_TEST.md
describe the ZED SDK 5.2.3 + CUDA setup on the *original x86 dev machine* (note
the `x86_64-linux` TensorRT paths there) — none of that is present on this Thor.
To use a ZED here you would, inside the container:

1. Install the **aarch64 / JetPack 7** build of the ZED SDK.
2. Re-clone the wrapper: `git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper ros2_ws/src/zed-ros2-wrapper` (it is gitignored).
3. `colcon build --packages-up-to zed_wrapper`.

Under CUDA 13 the patched cmake flag from STATE.md §8 is **not** needed (it was a
CUDA-12.5-only workaround).

## Common commands

```bash
./docker/run.sh                       # interactive shell (reuses a running container)
./docker/run.sh ./run_genesis.sh      # run the sim launcher directly
./docker/run.sh ./activate_nav2.sh    # force Nav2 lifecycle activation
docker exec -it g1_slam bash          # extra shell into the running container
```

## Am I inside the container?

Because the repo is mounted at the same path, the container user is also
`tracelab`, and `--network host` makes the container inherit the host's
hostname, the two shells look identical. After a rebuild the container prompt is
prefixed `(g1-docker)`. Regardless of prompt, the definitive check:

```bash
ls /.dockerenv && echo ">>> IN CONTAINER" || echo ">>> ON HOST"
grep VERSION_CODENAME /etc/os-release   # jammy = container, noble = host
printenv ROS_DISTRO                      # humble = container, empty = host
```

From the host, `docker ps` shows whether the `g1_slam` container is running.

## Troubleshooting

- **`rviz2` window doesn't appear** — run `xhost +local:root` on the host once,
  and confirm `echo $DISPLAY` is set (it is `:1` here).
- **Nodes on host and container don't see each other** — they share
  `--network host`; make sure `ROS_DOMAIN_ID` matches and `ROS_LOCALHOST_ONLY=0`.
- **Camera not found** — the container runs `--privileged` with `/dev` mounted;
  check `lsusb` inside the container and that the device is plugged into USB 3.
- **colcon build errors about `canonicalize_version` / numpy / matplotlib** —
  these are the STATE.md §8 gotchas; the image already applies the fixes
  (`packaging`, numpy 2, matplotlib upgrade). Rebuild if you edited deps.
