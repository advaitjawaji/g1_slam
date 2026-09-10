# G1 Human Avoidance — SLAM + Detection + Navigation

Autonomous navigation stack for the **Unitree G1 humanoid robot** that maps its
environment, detects humans, forecasts their trajectories, and drives the robot
around a room while stopping for anyone who enters its path.

Built with ROS 2 Humble, a **ZED 2i** stereo camera, YOLO26n human detection
(ported from `Human_dtp`), and Nav2. Supports hardware deployment, a camera-free
"blind nav" tier, Genesis simulation, and offline rosbag replay.

---

## Demo

https://github.com/user-attachments/assets/31447d2a-ec57-407d-af53-1afb119e20ef

The G1 navigating autonomously; a person crosses its path and it stops.

The clip is the output of `overlay_node` — the robot's own ZED view, annotated
with the detector's bounding boxes, each person's forecast trajectory, the Nav2
plan, and a bird's-eye inset. See [DEMO_VIDEO.md](DEMO_VIDEO.md) for how it is
produced and tuned.

### The configuration that recorded it

| Piece | What actually ran |
|---|---|
| Launcher | `WITH_ROBOT=1 NET_IFACE=<nic> USE_ZED_ODOM=1 ./run_zed_e2e.sh` |
| Camera | **ZED 2i**, `zed-ros2-wrapper` v5.3.1, ZED SDK 5.2.3, CUDA 13 |
| Odometry | **ZED 2i visual-inertial odometry** relayed onto `/odom`. RTAB-Map **off**. |
| Global frame | **`odom`** — there is no `map` frame in this wiring. Goals are sent in `odom`. |
| Costmaps | Rolling, fed by the ZED point cloud `/zed/zed_node/point_cloud/cloud_registered` |
| Detection | `detection_node` with `yolo26n.pt` on the live ZED RGB + depth |
| Avoidance | **Reactive stop** via `/g1/human_cmd` → `robot_node` velocity override |
| Locomotion | `robot_node` → `g1.py` → `unitree_sdk2py` `G1LocoClient.Move(vx, vy, wz)` |
| Video | `overlay_node` → `/g1/overlay/image` + MP4 in `~/g1_demos/` |

> **Humans are avoided by stopping, not by rerouting.** Predictive human
> obstacles are published on `/human_obstacle_cloud` but the costmap source is
> **commented out** in `nav2_params_hw.yaml` — see
> [Known limitations](#known-limitations). Nav2 does still reroute around
> whatever the ZED depth cloud sees, which includes the person's body.

---

## Architecture (as deployed)

```
ZED 2i
  │
  ├──► ZED VIO ──────────────────► /odom  +  TF odom → zed_camera_link → pelvis
  │
  ├──► point cloud ──────────────► Nav2 costmap obstacle layer (rolling, odom frame)
  │
  └──► RGB + depth ──► YOLO26n + depth back-projection + Kalman tracking
                          │
                          ├──► /humans/markers        (RViz)
                          ├──► /g1/detections         (JSON → overlay_node)
                          ├──► /human_obstacle_cloud  (PointCloud2, costmap source OFF)
                          └──► /g1/human_cmd  ── STOP / SLOW_DOWN / NORMAL_OPERATION
                                                        │
                     Nav2 ──► /cmd_vel ──► robot_node ──┤  velocity override
                                                        │  (×0.0 on STOP, ×0.4 on SLOW)
                                                        ▼
                                              g1.py → Unitree SDK → G1 walks
```

RTAB-Map is still wired in and is the **default** odometry source in
`run_zed_e2e.sh` (omit `USE_ZED_ODOM=1`). It was swapped for ZED VIO for the
demo because the IMU-fused estimate tracks a walking humanoid better than
frame-to-map visual odometry does.

---

## Validation status

The full stack has been deployed and validated on the **real Unitree G1 with a
ZED 2i and people moving in the scene**. The following were all confirmed on
hardware, not in simulation:

- **Human detection against real people** — `/g1/human_cmd` flips
  `NORMAL_OPERATION` → `SLOW_DOWN` → `STOP` as someone approaches
  ([ZED2_TEST.md](ZED2_TEST.md)).
- **ZED VIO → Nav2 → `/cmd_vel`** at ~20 Hz toward a goal in the `odom` frame,
  with the ZED point cloud driving the costmap obstacle layers.
- **RTAB-Map** as the alternative odometry source (`USE_ZED_ODOM` unset).
- **`robot_node` → `g1.py` → Unitree SDK** — the G1 takes real steps from
  `/cmd_vel`, and the `/g1/human_cmd` override zeroes them.
- **End-to-end with humans in the scene** — the robot walks toward its goal,
  halts when a person steps into its path, and resumes when they clear. That is
  the demo.
- **`overlay_node`** — annotated egocentric MP4 with AR paths and BEV inset,
  rendered live while the robot walked.

Genesis simulation and the offline rosbag path remain in the repo as
development and regression environments — they are how the integration was
brought up before hardware, and they still run.

> [STATE.md](STATE.md) predates hardware deployment and still describes the
> perception and SDK paths as unvalidated. Trust this section over it until it
> is refreshed.

---

## Documentation map

| File | What it is for |
|---|---|
| [README.md](README.md) | This file — architecture, packages, run modes. |
| [STATE.md](STATE.md) | Agent-oriented status: what is validated, gotchas, next steps. |
| [E2E_TEST.md](E2E_TEST.md) | **Staged bring-up** for the ZED hardware stack. Run this before the full stack. |
| [ZED2_TEST.md](ZED2_TEST.md) | Standalone ZED + detection lab procedure (no robot, no Nav2). |
| [DEMO_VIDEO.md](DEMO_VIDEO.md) | The annotated overlay: how to record it, tune it, and iterate off-robot. |
| [PLAN.md](PLAN.md) | Original 3-week implementation roadmap. |

---

## Repository Structure

```
g1_slam/
│
├── run_zed_e2e.sh                   ★ ZED hardware launcher (tmux, 4 windows) — the demo path
├── activate_nav2.sh                 Force Nav2 lifecycle active (autostart is flaky)
├── demo_tour.sh                     Hands-free multi-goal tour for recording
├── run_genesis.sh / stop_genesis.sh Genesis simulation launcher / teardown
├── test_offline.sh                  One-command offline rosbag pipeline test
├── install_ros2.sh                  ROS2 Humble + dependency installer
├── requirements.txt                 Python dependencies (non-ROS2)
│
├── tools/                           Off-robot validation — no ROS, numpy + OpenCV only
│   ├── preflight_overlay.sh         Pre-take check: which overlay layers you'll actually get
│   ├── preview_overlay.py           Synthetic scene → overlay MP4 + stills
│   ├── test_overlay_geometry.py     Pinhole / clipping checks
│   ├── test_detection_world_track.py  World-track correctness + camera-frame regression
│   └── test_detection_payload.py    The /g1/detections contract between the two nodes
│
├── genesis_sim/                     Genesis physics simulation bridge
│   ├── genesis_node.py              Main bridge: Genesis ↔ ROS2
│   ├── policy.py                    RL locomotion policy wrapper (Unitree G1)
│   └── g1_29dof.xml                 MuJoCo model (better physics than the URDF)
│
└── ros2_ws/
    ├── cmake/zed/                   Patched ZED cmake config (CUDA 12.5 fallback)
    └── src/
        ├── g1_description/          Robot model (URDF + 165 STL meshes)
        ├── g1_slam/                 RTAB-Map configuration
        ├── g1_detection/            Human detection + overlay pipeline
        ├── g1_robot/                G1 SDK locomotion + odom bridge
        └── g1_bringup/              Top-level launch files + Nav2 params
```

`ros2_ws/src/zed-ros2-wrapper/` is **gitignored** — clone it yourself, see
[Setup](#setup).

---

## ROS2 Packages

### `g1_description` — Robot Model

| File | Description |
|---|---|
| `urdf/g1_29dof.urdf` | Full 29-DOF G1 URDF with sensor frames |
| `urdf/meshes/` | 165 STL mesh files for all G1 links |
| `worlds/test_room.world` | 10×10 m Gazebo test room with obstacles |
| `launch/display.launch.py` | Visualise G1 in RViz with joint sliders |

Frames of interest:
- `pelvis` — robot base frame (URDF root, and Nav2's `robot_base_frame`)
- `d435_link` — RealSense D435 mount (pitched ~47° forward). **Unused on the ZED path.**
- `mid360_link` — Livox Mid360 LiDAR (available for future use)
- `imu_in_torso` — IMU frame

The **ZED is not in the URDF.** It is attached by a static transform published
by the launcher — see [Frames and TF](#frames-and-tf).

---

### `g1_slam` — RTAB-Map Configuration
Used on the Genesis/offline paths, and as the non-VIO odometry option on hardware.

| File | Description |
|---|---|
| `config/rtabmap.yaml` | RTAB-Map parameters (features, loop closure, grid map) |
| `launch/slam.launch.py` | Mapping mode |
| `launch/localization.launch.py` | Localization-only mode (uses an existing map) |

Key settings: `frame_id: pelvis`, `Grid/CellSize: 0.05`, `Vis/MinInliers: 15`,
`Rtabmap/DetectionRate: 1.0`, `Grid/3D: false` (2D occupancy grid for Nav2).

> On the ZED hardware path, `run_zed_e2e.sh` launches `rtabmap_launch` directly
> with `frame_id:=zed_camera_link` rather than using these files — RTAB-Map must
> be rooted at the camera, not the pelvis, or `rgbd_odometry` throws
> extrapolation warnings. See [E2E_TEST.md §6](E2E_TEST.md).

---

### `g1_detection` — Human Detection + Demo Overlay
Ported from the [Human_dtp](../Human_dtp) MiR AMR detection system.

| File | Description |
|---|---|
| `detection.py` | `HumanXZPredictor` — YOLO + depth + Kalman tracking + trajectory forecast |
| `detection_node.py` | ROS2 wrapper: camera-agnostic topic params, publishes markers/cmd/JSON |
| `human_obstacle_node.py` | Humans → `PointCloud2` for the Nav2 obstacle layer |
| `overlay_render.py` | Pure cv2/numpy renderer for the demo video (no ROS import) |
| `overlay_node.py` | Demo overlay node → `/g1/overlay/image` + MP4 |
| `kf.py` | Kalman filter used by the tracker |
| `preprocessing.py` | Depth frame temporal smoothing |
| `eigen.py` + `EigenTrajectory/` | Learned trajectory prediction (disabled by default) |

**Pipeline:**
1. YOLO26n detects humans (class 0) in the RGB image.
2. The tracker assigns a persistent `track_id` across frames.
3. Depth is back-projected to full XYZ using `camera_info` intrinsics, giving
   each person a real-world lateral X and forward Z.
4. An exponential moving average smooths noisy depth.
5. A physics model forecasts each person's position ~1.5 s ahead.
6. Region logic converts the nearest threat into `STOP` / `SLOW_DOWN` / `NORMAL_OPERATION`.

**Camera support** — every topic is a node parameter, so one node serves both:

| Camera | Color encoding | Depth encoding |
|---|---|---|
| ZED 2i | `bgra8` (alpha dropped) | `32FC1`, metres |
| RealSense D435i | `rgb8` / `bgr8` | `16UC1`, millimetres |

**Published topics:**

| Topic | Type | Contents |
|---|---|---|
| `/humans/markers` | `visualization_msgs/MarkerArray` | Red cylinders (current) + green lines (forecast) |
| `/g1/human_cmd` | `std_msgs/String` | `STOP` / `SLOW_DOWN` / `NORMAL_OPERATION` |
| `/human_obstacle_cloud` | `sensor_msgs/PointCloud2` | Current + forecast human positions as costmap obstacles |
| `/g1/detections` | `std_msgs/String` (JSON) | Per-frame bboxes, track IDs, world positions, forecasts, source-frame stamp |
| `/g1/overlay/image` (+ `/compressed`) | `sensor_msgs/Image` | The annotated demo video |

`overlay_node` consumes `/g1/detections` rather than running YOLO a second time,
and draws each result on the frame whose timestamp it was computed from — so
boxes sit on the pixels YOLO actually saw instead of lagging by the inference
latency.

**Avoidance thresholds** (`detection.py` defaults):

| Zone | Distance | Action |
|---|---|---|
| Front slow | < 3.8 m | SLOW_DOWN (40% speed) |
| Front stop | < 1.8 m | STOP |
| Side slow | < 2.0 m | SLOW_DOWN |
| Side stop | < 0.8 m | STOP |
| Immediate | < 0.8 m any direction | Emergency STOP |

---

### `g1_robot` — G1 SDK Interface

| File | Description |
|---|---|
| `robot_interface.py` | Abstract base class defining the robot interface |
| `g1.py` | `G1Robot` — wraps `G1LocoClient` from `unitree_sdk2py` |
| `robot_node.py` | Subscribes `/cmd_vel` + `/g1/human_cmd`, drives the robot |
| `odom_bridge.py` | Camera-free odometry from the G1's own state (Tier-3 nav) |
| `launch/robot.launch.py` | Launch the robot node standalone |

**Control flow:**
```
Nav2 /cmd_vel ──► robot_node ──► G1Robot.send_command()
                      │                    │
              /g1/human_cmd          G1LocoClient.Move(vx, vy, wz)
              (velocity scale)             │
                                     G1 walks
```

**Safety override:** `STOP` sets the velocity scale to `0.0` — every Nav2
command is zeroed regardless of what Nav2 wants. `SLOW_DOWN` sets it to `0.4`.
`NORMAL_OPERATION` restores `1.0`.

**Stub mode:** if the Unitree SDK is missing or the G1 is unreachable, `G1Robot`
prints commands instead of sending them. The log line to check is
`Connected via DDS` — if you see `[G1 STUB]`, the robot will not move.

`robot_node` does **not** stand the robot up. Stand it with the remote (or the
SDK's `StandUp`) before sending goals.

**`net_iface`** selects the host NIC on the robot's subnet for DDS. Find it with:
```bash
ip -br addr | awk '/192\.168\.123\./{print $1}'
```

---

### `g1_bringup` — Launch Files and Configuration

| File | Description |
|---|---|
| `launch/hardware.launch.py` | Full hardware stack in one launch file (`camera:=realsense\|zed`) |
| `launch/nav_tier3.launch.py` | **Camera-free** autonomous nav — `odom_bridge` + Nav2 + robot_node |
| `launch/genesis.launch.py` | Genesis simulation mode |
| `launch/offline.launch.py` | Rosbag offline pipeline test |
| `launch/sim.launch.py` | Gazebo Classic (camera limited by driver issues) |
| `config/nav2_params_hw.yaml` | **Hardware** Nav2 params — obstacle-aware, odom-frame, rolling |
| `config/nav2_params_sim.yaml` | **Sim** Nav2 params — permissive, collision detection off |

> Keep the two param files apart. The sim file disables collision detection and
> unknown-space tracking to cope with a featureless simulated room; running it on
> hardware would walk the robot into things.

**`nav2_params_hw.yaml` highlights:**

| Setting | Value | Why |
|---|---|---|
| `global_frame` | `odom` | No `map` frame is published on the ZED VIO path |
| `robot_base_frame` | `pelvis` | URDF root |
| `odom_topic` | `/odom` | ZED VIO, relayed |
| Global costmap | rolling, 20×20 m | No persistent map to plan over |
| Local costmap | rolling, 4×4 m | |
| Obstacle source | `/zed/zed_node/point_cloud/cloud_registered` | `PointCloud2` — Nav2 accepts nothing else |
| `robot_radius` | 0.30 m | |
| Controller | Regulated Pure Pursuit, `desired_linear_vel: 0.5` | |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.2 m / 0.50 rad | Loose enough that the G1 stops instead of circling |
| `use_collision_detection` | `true` | Hardware only |
| Human obstacle source | **commented out** | See [Known limitations](#known-limitations) |

---

## Run Modes

### Mode 1 — ZED 2i hardware, end-to-end ★ (the demo path)

**Do the staged bring-up first.** [E2E_TEST.md](E2E_TEST.md) walks the stack up
one layer at a time — locomotion, blind nav, camera, detection, TF, Nav2, then
everything — with a pass check for each. A failure is far easier to find in its
own stage than inside the full stack.

Then the one-shot launcher:

```bash
cd ~/Desktop/g1_slam

# Perception only — no robot motion, safe
./run_zed_e2e.sh

# ZED VIO odometry instead of RTAB-Map (what the demo used)
USE_ZED_ODOM=1 ./run_zed_e2e.sh

# Full demo: camera + Nav2 + the real robot walking
WITH_ROBOT=1 NET_IFACE=<nic> USE_ZED_ODOM=1 ./run_zed_e2e.sh

# Tear down — ALWAYS via the script, so the MP4 is flushed and closed
./run_zed_e2e.sh stop
```

**Environment toggles:**

| Variable | Default | Effect |
|---|---|---|
| `WITH_ROBOT` | `0` | `1` starts `robot_node` — **the G1 will walk** |
| `NET_IFACE` | auto | Host NIC on the robot subnet for DDS |
| `USE_ZED_ODOM` | `0` | `1` = ZED VIO, RTAB-Map off. `0` = RTAB-Map visual odometry |
| `OVERLAY` | `1` | The annotated demo video node |
| `RECORD` | `1` | Write the overlay to an MP4 |
| `DEMO_DIR` | `~/g1_demos` | Where the MP4 lands |

**tmux layout** (`tmux attach -t g1_zed`; `Ctrl-b n` / `Ctrl-b p` switch windows):

| Window | Panes |
|---|---|
| `core` | ZED wrapper · odom source (RTAB-Map or ZED relay) · detection_node · Nav2 + lifecycle activate |
| `extras` | static TF `zed_camera_link→pelvis` · robot_state_publisher · RViz · monitor shell |
| `ctrl` | human_obstacle_node · robot_node (only with `WITH_ROBOT=1`) |
| `demo` | overlay_node (only with `OVERLAY=1`) |

Startup delays (~6–18 s) are baked in so each layer sees the topics it needs.

**Send a goal** — `frame_id` **must be `odom`**, there is no `map` frame:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'},
           pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

Or **2D Goal Pose** in RViz with Fixed Frame set to `odom`.

**Watch the loop:**
```bash
ros2 topic echo /g1/human_cmd     # STOP / SLOW_DOWN / NORMAL_OPERATION
ros2 topic hz   /cmd_vel          # ~20 Hz while a goal is active
ros2 run rqt_image_view rqt_image_view /g1/overlay/image
```

**Safety:** clear the space, keep the e-stop in hand, and have the robot already
STANDING. `robot_node` will not stand it for you.

---

### Mode 2 — `hardware.launch.py` (single launch file)

The same components as Mode 1 in one launch file, with staged `TimerAction`
delays instead of tmux panes.

```bash
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-nav2-regulated-pure-pursuit-controller

source /opt/ros/humble/setup.bash
source ~/Desktop/g1_slam/ros2_ws/install/setup.bash

ros2 launch g1_bringup hardware.launch.py camera:=zed robot_ip:=192.168.123.161
```

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `camera` | `realsense` | `realsense` or `zed` |
| `robot_ip` | `192.168.123.161` | G1 IP address |
| `net_iface` | *(auto)* | Host NIC on the robot subnet for DDS |
| `model_path` | `yolo11n.pt` | YOLO model — pass `yolo26n.pt`, or `yolo26n.engine` on Jetson |
| `rviz` | `true` | Launch RViz |
| `slam_mode` | `mapping` | `mapping` or `localization` |
| `overlay` | `true` | Annotated demo video on `/g1/overlay/image` |
| `record_video` | `true` | Write the overlay to an MP4 |
| `video_path` | *(auto)* | Empty ⇒ `~/g1_demos/g1_demo_<timestamp>.mp4` |

**Launch sequence:** camera + robot_state_publisher at t=0, SLAM at t=0,
robot_node at t=5 s, detection + human_obstacle at t=8 s, overlay at t=10 s,
Nav2 at t=12 s.

> ⚠️ **This is not the path the demo was recorded on.** Its ZED topic remaps
> (`zed_node/rgb/image_rect_color`, `zed_node/rgb/camera_info`) predate wrapper
> v5.3.1, which publishes `rgb/color/rect/image` and `rgb/color/rect/camera_info`.
> Verify against `ros2 topic list | grep zed` before relying on it. It also
> launches RTAB-Map unconditionally, whereas the demo ran ZED VIO with RTAB-Map
> off. Use `run_zed_e2e.sh` for the known-good path.

---

### Mode 3 — Camera-free "blind" navigation (Tier 3)

Autonomous go-to-pose with **no camera at all**. Localization comes from the
G1's own motion estimate via `odom_bridge`, which picks the best available
source: SDK `SportModeState` → IMU yaw + `cmd_vel` → pure dead reckoning. A
static identity `map→odom` puts the robot at the origin, so goals go in the
**`map`** frame here.

```bash
ros2 launch g1_bringup nav_tier3.launch.py net_iface:=<nic>

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

> ⚠️ **No obstacle sensing and no human detection.** The robot walks blind.
> Clear, supervised space and e-stop only. This is Stage 2 of
> [E2E_TEST.md](E2E_TEST.md) — it isolates Nav2 + SDK from every perception
> problem, which is exactly what you want when the full stack misbehaves.

---

### Mode 4 — Genesis Simulation

Physics simulation with the G1 **MuJoCo model** (`genesis_sim/g1_29dof.xml`),
scripted human actors, and a virtual RGB-D camera, feeding the same SLAM and
Nav2 stack.

The MuJoCo model is used instead of the URDF because it has exact foot contact
geometry, correct joint definitions, and is the format the Unitree RL locomotion
policy was trained against.

```bash
pip3 install genesis-world
pip3 install "coverage>=7.0"        # fixes the numba import error

./run_genesis.sh                     # kinematic — robot slides, no leg motion
./run_genesis.sh --viewer            # GPU viewer window (RTX required)
./run_genesis.sh --policy /path/to/g1_policy.pt   # RL walking policy
./stop_genesis.sh
```

Nav2's lifecycle autostart is unreliable here — force it:
```bash
./activate_nav2.sh
for n in controller_server planner_server bt_navigator; do
  echo -n "$n: "; ros2 lifecycle get /$n 2>/dev/null | grep -v XMLPARSER | head -1
done
```

Send goals in the `odom` frame (`map` and `odom` coincide in sim), or run
`./demo_tour.sh` for a hands-free multi-goal tour that weaves through both
scripted humans.

**What is and isn't the same as hardware:**

| Component | Same? | Notes |
|---|---|---|
| Nav2 planner + controller | ⚠️ Same code | Different params — sim uses the permissive file |
| Avoidance thresholds + override | ✅ Yes | Same numbers, same `/g1/human_cmd` contract |
| SLAM (RTAB-Map) | ✅ Yes | Same code, same config |
| Human detection (YOLO) | ❌ No | `genesis_node` publishes ground-truth positions — cylinders aren't detectable by YOLO |
| Low-level locomotion | ❌ No | Kinematic slide or RL policy — not Unitree's onboard MPC |

**Genesis cameras — two separate ones:**

| Camera | Purpose | Configuration |
|---|---|---|
| `scene.add_camera()` | The robot's sensor → `/camera/color/image_raw` | At the D435 mount: `xyz=(0.0576, 0.0175, 1.267)`, pitch 47.6° |
| `viewer_options` | Spectator view in the `--viewer` window | `camera_pos=(5, -5, 4)` looking at origin |

**Genesis gotchas (all already fixed in the repo):**

| Symptom | Cause | Fix applied |
|---|---|---|
| `numba` import error | coverage version conflict | `pip3 install "coverage>=7.0"` |
| `add_entity() got unexpected keyword 'pos'` | Genesis 1.0 API | `pos` goes inside the morph |
| `camera.render() too many values to unpack` | Genesis 1.0 API | Returns 4 values: `rgb, depth, _, _` |
| Robot moves in reverse | Quaternion order | Genesis is `(w, x, y, z)`, not `(x, y, z, w)` |
| First goal works, later goals fail | Two publishers on `map→odom` | `publish_tf: false` on the rtabmap node |
| `follow_path` aborts, no `/cmd_vel` | RPP collision detection + unknown-as-lethal | `use_collision_detection: false`, `track_unknown_space: false` |
| Figure-8, never stops at goal | Strict yaw tolerance | `yaw_goal_tolerance: 3.15` (position-only checker) |
| Won't turn around for a goal behind | Rotate-to-heading off | `use_rotate_to_heading: true` with loose yaw tolerance |
| Camera ~1 Hz, SLAM starves | CPU rendering 640×480 | Camera at 160×120, larger RTAB-Map sync queue |

**RL policy** (`genesis_sim/policy.py`) wraps Unitree's pre-trained locomotion
policy: a 53-dim observation (IMU angular velocity, gravity vector, cmd_vel, 15
joint positions, 15 velocities, 15 previous actions, 4 gait phase) → 15-dim
joint position offsets, at 50 Hz. It falls back to a PD standing controller with
no checkpoint. Get one from
[`unitree_rl_gym`](https://github.com/unitreerobotics/unitree_rl_gym)
(`logs/g1/policy.pt`).

---

### Mode 5 — Offline Rosbag Replay

Validates perception + planning against pre-recorded data. No hardware, no
simulation.

**Record:**
```bash
ros2 bag record -o ~/lab_test \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info \
  /camera/depth/points \
  /imu_in_torso/data \
  /tf /tf_static
```
Walk around the room, then walk in front of the camera. 2–3 minutes is plenty.

**Replay:**
```bash
./test_offline.sh ~/lab_test
```

Or manually: `ros2 launch g1_bringup offline.launch.py` in one terminal,
`ros2 bag play ~/lab_test --clock --rate 1.0` in another.

Tests SLAM, detection, human→costmap conversion, path planning, and the velocity
override. Does **not** test robot walking.

---

## Frames and TF

On the deployed ZED VIO path there is exactly one tree, and **no `map` frame**:

```
odom ──► zed_camera_link ──► pelvis ──► (URDF: torso, limbs, ...)
  │              │
  │              └──► zed_left_camera_frame ──► ..._optical
  │
  └──► ZED VIO publishes odom → zed_camera_link
```

The ZED wrapper hard-codes its base to `<camera>_camera_link` — there is **no
`base_frame` parameter** — so it always publishes `odom → zed_camera_link`.
`pelvis` is therefore hung *underneath* the camera by a static transform, which
is legal because `pelvis` is the URDF root and free to take a parent.

**The mount:** ZED on the head, ~5 cm forward of and ~40 cm above the pelvis,
pitched ~17.5° (0.3054 rad) **down**. The published transform is the inverse of
that, because it goes camera → pelvis:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0726 --y 0.0 --z -0.3965 --yaw 0 --pitch -0.3054 --roll 0 \
  --frame-id zed_camera_link --child-frame-id pelvis
```

`t = -Ry(-0.3054) · (0.05, 0, 0.40) = (0.0726, 0, -0.3965)`. If you re-measure
the pitch, recompute `t` — the same values appear in both `run_zed_e2e.sh` and
`hardware.launch.py`'s `zed_to_pelvis_tf`, and must stay in sync.

**Verify before every session that matters:**
```bash
ros2 run tf2_tools view_frames     # ONE tree, no frame with two parents
ros2 run tf2_ros tf2_echo odom pelvis
```
Two publishers on the same edge is the single most common cause of "the robot
navigates to one goal and then stops accepting them."

> **The floor is not `z = 0`.** With ZED VIO the `odom` origin sits wherever the
> camera was when tracking started — roughly 1.2 m up. The overlay derives the
> floor as `pelvis_z − pelvis_height_m` (default 0.75). See
> [DEMO_VIDEO.md](DEMO_VIDEO.md) if the AR paths float or sink.

---

## Key Topics Reference

**ZED path (as deployed):**

| Topic | Type | Published by |
|---|---|---|
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/Image` | zed_wrapper (`bgra8`) |
| `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` | zed_wrapper (`32FC1`, metres) |
| `/zed/zed_node/rgb/color/rect/camera_info` | `sensor_msgs/CameraInfo` | zed_wrapper |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | zed_wrapper — the costmap obstacle source |
| `/zed/zed_node/odom` | `nav_msgs/Odometry` | ZED VIO |
| `/odom` | `nav_msgs/Odometry` | `topic_tools relay` (ZED VIO) or RTAB-Map |

**Stack topics:**

| Topic | Type | Published by | Description |
|---|---|---|---|
| `/humans/markers` | `visualization_msgs/MarkerArray` | detection_node | Positions + forecasts |
| `/g1/human_cmd` | `std_msgs/String` | detection_node | `STOP` / `SLOW_DOWN` / `NORMAL_OPERATION` |
| `/g1/detections` | `std_msgs/String` (JSON) | detection_node | Full per-frame result for the overlay |
| `/human_obstacle_cloud` | `sensor_msgs/PointCloud2` | human_obstacle_node | Humans as costmap obstacles (source disabled) |
| `/g1/overlay/image` | `sensor_msgs/Image` | overlay_node | Annotated demo video |
| `/plan`, `/local_plan` | `nav_msgs/Path` | Nav2 | Planned path |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 | Velocity commands |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map | 2D occupancy grid — **not published on the ZED VIO path** |

**RealSense path** (`camera:=realsense`): `/camera/color/image_raw`,
`/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info`,
`/camera/depth/points`, `/imu_in_torso/data`.

---

## Setup

### 1. ROS 2 Humble

```bash
cd ~/Desktop/g1_slam
chmod +x install_ros2.sh
./install_ros2.sh
source ~/.bashrc
```

### 2. ROS packages

```bash
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-nav2-regulated-pure-pursuit-controller \
                 ros-humble-rtabmap-ros \
                 ros-humble-topic-tools \
                 ros-humble-realsense2-camera      # RealSense path only
```
`ros-humble-topic-tools` is required for the `USE_ZED_ODOM=1` relay.

### 3. Python dependencies

```bash
pip3 install -r requirements.txt   # ultralytics, torch, opencv-python, lap, numpy, ...
pip3 install -U packaging          # else colcon fails on canonicalize_version
pip3 install genesis-world         # Genesis simulation only
pip3 install unitree_sdk2py        # hardware only (or clone from GitHub)
```

### 4. ZED SDK + ROS 2 wrapper

The wrapper is **gitignored** — clone it into the workspace yourself:

```bash
# ZED SDK 5.2.3 from stereolabs.com, then:
cd ~/Desktop/g1_slam/ros2_ws/src
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
```

Verify the SDK before touching ROS — camera problems are much easier to
diagnose outside it:
```bash
/usr/local/zed/tools/ZED_Diagnostic     # expect "ZED SDK Diagnostic : OK"
/usr/local/zed/tools/ZED_Explorer       # expect live video
```
First-ever use on a machine downloads a factory calibration file — **the one
step that needs internet.** Run `ZED_Explorer` once somewhere with wifi if the
lab has none. Use a **USB 3** port (blue tab).

> **CUDA 12.5 only?** ZED SDK 5.2.3's cmake demands the CUDA 13 toolkit even
> though the SDK binary needs only the driver. A patched config lives at
> `ros2_ws/cmake/zed/`:
> ```bash
> colcon build --packages-up-to zed_wrapper --cmake-args -DZED_DIR=$PWD/cmake/zed
> ```
> Under CUDA 13 this flag is unnecessary.

### 5. Build the workspace

```bash
cd ~/Desktop/g1_slam/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Every terminal that runs the ZED stack also needs the TensorRT loader path:
```bash
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/targets/x86_64-linux/lib:$LD_LIBRARY_PATH
```

### 6. YOLO weights

Place `yolo26n.pt` in the repo root (~5.3 MB). Weights are gitignored; caching
them locally means the demo needs no internet. On Jetson, export to TensorRT and
pass `model_path:=yolo26n.engine`.

### 7. Verify

```bash
ros2 pkg list | grep g1                      # 5 packages
ros2 pkg list | grep zed                     # 4 packages
ros2 pkg executables g1_detection            # detection_node, human_obstacle_node, overlay_node
python3 -c "from g1_detection.detection_node import DetectionNode; print('OK')"
python3 tools/test_detection_payload.py      # no ROS needed
```

---

## Hardware Requirements

| Component | Specification |
|---|---|
| Robot | Unitree G1 (29 DOF), standing, e-stop in hand |
| Camera | **ZED 2i** on a head mount (RealSense D435i supported as an alternative) |
| Compute | Ubuntu 22.04 + ROS 2 Humble, NVIDIA GPU with CUDA 13 (12.5 with the patched cmake) |
| Network | G1 on `192.168.123.0/24`, static IP `192.168.123.161` |
| USB | ZED on a USB 3 port — USB 2 enumerates but streams fail |

The G1 also carries a RealSense D435i (`d435_link`), two IMUs (`imu_in_torso`,
`imu_in_pelvis`), and a Livox Mid360 LiDAR (`mid360_link`, unused). The MuJoCo
model defines only mechanical structure — sensor frames live in the URDF.

---

## RViz

`run_zed_e2e.sh` opens RViz with the Nav2 default preset. Adjust it once and
save your own config:

1. **Global Options → Fixed Frame** = `odom` (there is no `map` frame)
2. **Views → Type** = `Orbit` (the default `TopDownOrtho` gives top-down only)

| Display | Topic | Shows |
|---|---|---|
| PointCloud2 | `/zed/zed_node/point_cloud/cloud_registered` | Live ZED cloud |
| Image | `/g1/overlay/image` | The annotated demo view |
| MarkerArray | `/humans/markers` | Red cylinders = humans, green lines = forecasts |
| RobotModel | `/robot_description` | G1 model |
| Path | `/plan` | Nav2 planned path |
| Map | `/map` | RTAB-Map grid (RTAB-Map odometry mode only) |

Two markers per human is by design: `ns=humans` (cylinder) and
`ns=human_predictions` (line). Toggle namespaces in the display.

---

## Known limitations

These are deliberate scope boundaries of the deployed system, not gaps in what
was tested — the stack below is the one that ran on the robot.

**Predictive human costmap injection is disabled.** `human_obstacle_node`
publishes `/human_obstacle_cloud` as a valid `PointCloud2`, and
`nav2_params_hw.yaml` has the `humans` observation source written out — but
commented out. The reason is a coordinate bug: `_camera_xz_to_odom_xz` in
`detection.py` drops the optical Y axis and then reads world rows 0 and 2, so it
treats world *height* as a ground axis and applies a body-frame transform to
optical-frame points. That is harmless while the camera is level and TF is near
identity — which is why simulation passed — but it skews human world positions
on a walking robot. Enabling the costmap source would mark obstacles in the
wrong place.

Fixing it properly means re-tuning the front/side region logic that drives the
safety `STOP`, so it was deliberately left alone. `detection.py` already carries
a second, geometrically correct track (`tracks_world`, a full 4×4 transform of
the camera *optical* frame into the odom ground plane) that the overlay uses —
**that is the version to switch the costmap path over to.**
`tools/test_detection_world_track.py` verifies the legacy path is bit-identical,
so the safety behaviour did not change.

Consequence for the demo: **the robot stops for people, it does not plan around
them.** Reactive avoidance via `/g1/human_cmd` is unaffected and works.

**No persistent global map on the ZED VIO path.** Both costmaps are rolling in
`odom` by design. Goals are relative to where tracking started, and there is no
loop closure correcting drift. Static-map planning would mean setting
`global_costmap.global_frame: map`, adding `static_layer` back, and running
RTAB-Map (`./run_zed_e2e.sh` without `USE_ZED_ODOM=1`) — a different
configuration from the one deployed.

**Forecast lines need a moving person.** `HumanXZPredictor` discards heading
samples below 0.08 m of travel and returns nothing with fewer than two
survivors, so a person standing still produces boxes but no magenta forecast.
That is the model's behaviour, not a bug. It also needs ~3 frames of track
history before forecasting at all.

**`hardware.launch.py`'s ZED topic remaps are stale** relative to wrapper
v5.3.1 — see Mode 2.

---

## Troubleshooting

**ZED / CUDA**

| Symptom | Fix |
|---|---|
| `CORRUPTED SDK INSTALLATION` / `NEURAL CORRUPTED MODEL` | `sudo /usr/local/zed/tools/ZED_Diagnostic -aio && sudo /usr/local/zed/tools/ZED_Diagnostic -nrlo_all` |
| `Unable to load library: libnvinfer_builder_resource.so.10.13.2` | `export LD_LIBRARY_PATH=/usr/local/cuda-13.0/targets/x86_64-linux/lib:$LD_LIBRARY_PATH` |
| `Camera model does not match user parameter 'zed2i'` | Launch with `camera_model:=zed2i`, not `zed2` |
| `Gravity alignment issues detected` | IMU re-aligning — hold the camera still ~2 s at startup |
| `zed_components: could NOT find CUDA: required is at least 13` | Build with `-DZED_DIR=$PWD/cmake/zed` (patched config) |
| `package 'zed_wrapper' not found` | Terminal didn't `source ros2_ws/install/setup.bash` |
| `CAMERA STREAM FAILED TO START` | USB 2 port, bad cable, or camera not enumerated — check `lsusb \| grep -i stereolabs` |

**Nav2 / TF**

| Symptom | Fix |
|---|---|
| Nav2 won't activate: `clear_entirely_global_costmap not available` | A costmap failed to init. Most often an `observation_sources` entry with `data_type: "PointCloud"` — Nav2 only accepts `PointCloud2`/`LaserScan`. |
| `Node not recognized: <X>` in the BT XML | Add `nav2_<X>_bt_node` to `bt_navigator.plugin_lib_names` |
| `Robot is out of bounds of the costmap!` | Global costmap empty or static-only. It should be rolling, 20×20 m, on `odom`. |
| Goal returns `distance_remaining: 0.0` immediately | Empty costmap, or the goal is inside `xy_goal_tolerance`. Move the camera ~15 s to seed the cloud, then send a goal 3+ m out. |
| First goal works, later ones fail | Two publishers on `map→odom` (or `odom→pelvis`). `ros2 run tf2_tools view_frames` — no frame may have two parents. |
| `rgbd_odometry` extrapolation warnings on `pelvis → ..._optical` | RTAB-Map launched with `frame_id:=pelvis`. Use `frame_id:=zed_camera_link`. |
| `Could not resolve host: meshes` in RViz | Wrong URDF. Use the packaged one: `$(ros2 pkg prefix g1_description)/share/g1_description/urdf/g1_29dof.urdf` |
| Lifecycle nodes stuck inactive | `./activate_nav2.sh` — autostart is unreliable |

**Robot**

| Symptom | Fix |
|---|---|
| `[G1 STUB]` in the log, no motion | SDK import failed or the G1 is unreachable. `ping 192.168.123.161`; check `net_iface`. |
| Connected but no step | Robot not in a walking-ready stance — re-stand it with the remote |
| Verify the SDK API | `python3 -c "import unitree_sdk2py.g1.loco.g1_loco_client as m; print(dir(m))"` |

**Detection / overlay**

| Symptom | Fix |
|---|---|
| Boxes render but no AR layers | The `odom → <optical frame>` TF doesn't resolve; the node logs `No TF 'odom' <- '...'` once. Override with `-p optical_frame:=<frame>`. |
| Boxes but no forecast lines | The person is standing still — see Known limitations |
| MP4 won't open | It's finalised on `overlay_node` shutdown. Always stop with `./run_zed_e2e.sh stop` (SIGINT first), never SIGKILL. |
| Overlay text shows `?` | OpenCV Hershey fonts are ASCII-only. Keep `title` ASCII. |
| `numpy.core.multiarray failed to import` | `sudo pip3 install --upgrade matplotlib` |
| EigenTrajectory import error | Handled gracefully — it is disabled by default |
| gzserver crashes on launch | NVIDIA 595+ vs Gazebo Classic. Use Genesis or the offline bag instead. |

Run `./tools/preflight_overlay.sh` after the stack settles (~25 s) — it checks
camera + intrinsics, the AR transform, detection flow, forecasts, `/plan`, and
output rate, in the order things actually fail.

---

## Relationship to Human_dtp

This repo builds on [`Human_dtp`](../Human_dtp), built for a **MiR AMR (wheeled
robot)**. The detection pipeline (`detection.py`, `eigen.py`, `kf.py`,
`preprocessing.py`, `EigenTrajectory/`) is ported largely intact.

| Human_dtp | g1_slam |
|---|---|
| MiR AMR (wheeled) | Unitree G1 (humanoid) |
| ZED 2i camera | **ZED 2i camera** (RealSense D435i also supported) |
| ZMQ transport | ROS 2 topics |
| Docker on Jetson | ROS 2 workspace |
| REST API robot control | Unitree SDK velocity interface |
| No SLAM (fixed route) | RTAB-Map SLAM / ZED VIO |
| No Nav2 | Nav2 autonomous navigation |
| Camera-frame tracks only | Camera-frame tracks + world-frame tracks (overlay) |

Additions on top of the port: depth back-projection to full XYZ from
`camera_info` intrinsics, camera-agnostic topic parameters, ZED `bgra8` /
`32FC1` handling, the `/g1/detections` JSON contract, and `tracks_world`.

---

## Implementation Plan

[`PLAN.md`](PLAN.md) holds the original 3-week roadmap. [`STATE.md`](STATE.md)
§9 holds the live next-steps list.
