# G1 Human Avoidance — SLAM + Detection + Navigation

Autonomous navigation stack for the **Unitree G1 humanoid robot** that simultaneously maps its environment, detects humans, predicts their trajectories, and plans collision-free paths around them in real time.

Built with ROS2 Humble, RTAB-Map, YOLO, and Nav2. Supports three execution modes: Genesis simulation, offline rosbag validation, and real hardware deployment.

---

## Demo Goal

The G1 walks autonomously through a room. When a human enters its path, the robot slows down, reroutes around them, and resumes its goal — all without any human operator input.

```
RealSense D435i
      │
      ├──► RTAB-Map SLAM ──────────────► /map  (room map builds in real time)
      │
      └──► YOLO + Depth ──────────────► Human positions in world frame
                                              │
                              ┌───────────────┴───────────────┐
                              ▼                               ▼
                    Nav2 costmap injection          Velocity safety override
                    (path reroutes around humans)   (hard stop if too close)
                              │                               │
                              └──────────── Nav2 ─────────────┘
                                              │
                                         /cmd_vel
                                              │
                                     Unitree G1 SDK
                                              │
                                       G1 walks
```

---

## Repository Structure

```
g1_slam/
│
├── PLAN.md                          Detailed 3-week implementation plan
├── README.md                        This file
├── requirements.txt                 Python dependencies (non-ROS2)
├── install_ros2.sh                  ROS2 Humble + all dependencies install script
├── run_genesis.sh                   Single-command Genesis simulation launcher (tmux)
├── stop_genesis.sh                  Kill the Genesis simulation session
├── test_offline.sh                  One-command offline rosbag pipeline test
│
├── genesis_sim/                     Genesis physics simulation bridge
│   ├── genesis_node.py              Main bridge: Genesis ↔ ROS2
│   ├── policy.py                    RL locomotion policy wrapper (Unitree G1)
│   └── g1_29dof.xml                 MuJoCo model (used by Genesis — better physics than URDF)
│
└── ros2_ws/src/
    ├── g1_description/              Robot model
    ├── g1_slam/                     SLAM configuration
    ├── g1_detection/                Human detection pipeline
    ├── g1_robot/                    G1 SDK locomotion interface
    └── g1_bringup/                  Top-level launch files
```

---

## ROS2 Packages

### `g1_description` — Robot Model
Contains the Unitree G1 URDF, all STL mesh files, and the Gazebo simulation world.

| File | Description |
|---|---|
| `urdf/g1_29dof.urdf` | Full 29-DOF G1 URDF with sensor frames and Gazebo plugins |
| `urdf/meshes/` | 165 STL mesh files for all G1 links |
| `worlds/test_room.world` | 10×10m Gazebo test room with obstacles |
| `launch/display.launch.py` | Visualise G1 in RViz with joint sliders |

Key frames in the URDF:
- `pelvis` — robot base frame (root link)
- `d435_link` — RealSense D435 camera (pitched ~47° forward from torso)
- `mid360_link` — Livox Mid360 LiDAR (available for future use)
- `imu_in_torso` — IMU used by SLAM

---

### `g1_slam` — SLAM Configuration
RTAB-Map configuration tuned for RealSense D435i on a walking humanoid.

| File | Description |
|---|---|
| `config/rtabmap.yaml` | RTAB-Map parameters (feature detection, loop closure, grid map) |
| `launch/slam.launch.py` | Launch SLAM in mapping mode |
| `launch/localization.launch.py` | Launch SLAM in localization-only mode (uses existing map) |

Key RTAB-Map settings:
- `frame_id: pelvis` — robot base frame
- `Grid/CellSize: 0.05` — 5cm occupancy grid resolution
- `Vis/MinInliers: 15` — minimum feature matches for loop closure
- `Rtabmap/DetectionRate: 1.0` — map update rate (Hz)

---

### `g1_detection` — Human Detection Pipeline
Ported from the [Human_dtp](../Human_dtp) MiR AMR detection system, adapted for G1 + ROS2.

| File | Description |
|---|---|
| `detection.py` | `HumanXZPredictor` — YOLO + depth + Kalman tracking + trajectory prediction |
| `detection_node.py` | ROS2 node wrapping the detector: subscribes to camera, publishes markers |
| `human_obstacle_node.py` | Converts human positions to Nav2 costmap obstacles |
| `overlay_render.py` | Pure cv2/numpy renderer for the demo video (no ROS import) |
| `overlay_node.py` | Demo overlay: annotated egocentric video + MP4 — see [DEMO_VIDEO.md](DEMO_VIDEO.md) |
| `eigen.py` | EigenTrajectory adapter (learned trajectory prediction, disabled by default) |
| `preprocessing.py` | Depth frame temporal smoothing |

**Detection pipeline internals:**
1. YOLO detects humans (class 0) in the RGB image
2. Tracker assigns persistent `track_id` to each person across frames
3. RealSense point cloud gives real-world X/Z position for each bounding box
4. Exponential Moving Average smooths noisy depth measurements
5. Physics-based model predicts each human's position 1.5 seconds ahead
6. Predicted positions fed to Nav2 costmap — robot reroutes proactively

**Published topics:**
- `/humans/markers` — `visualization_msgs/MarkerArray` of current positions (red cylinders) and predicted trajectories (green lines)
- `/g1/human_cmd` — `std_msgs/String` of `STOP`, `SLOW_DOWN`, or `NORMAL_OPERATION`
- `/human_obstacle_cloud` — `sensor_msgs/PointCloud` of human positions for Nav2 obstacle layer
- `/g1/detections` — `std_msgs/String` (JSON): full per-frame result — bounding
  boxes, track IDs, world positions, forecast trajectories, stamped with the
  source frame. Consumed by `overlay_node` so the demo video shows exactly the
  detections the avoidance logic acted on, with no second YOLO pass.
- `/g1/overlay/image` (+ `/compressed`) — the annotated demo video

**Avoidance thresholds (configurable in detection_node params):**

| Zone | Distance | Action |
|---|---|---|
| Front stop | < 1.8m | STOP |
| Front slow | < 3.8m | SLOW DOWN (40% speed) |
| Side stop | < 0.8m | STOP |
| Immediate | < 0.8m any direction | Emergency STOP |

---

### `g1_robot` — G1 SDK Interface
Translates ROS2 `/cmd_vel` commands to Unitree G1 locomotion SDK calls.

| File | Description |
|---|---|
| `robot_interface.py` | Abstract base class defining the robot interface |
| `g1.py` | `G1Robot` — implements `G1LocoClient` from `unitree_sdk2py` |
| `robot_node.py` | ROS2 node: subscribes to `/cmd_vel` + `/g1/human_cmd`, drives robot |
| `launch/robot.launch.py` | Launch the robot node |

**Control flow:**
```
Nav2 /cmd_vel ──► robot_node ──► G1Robot.send_command()
                      │                    │
              /g1/human_cmd          G1LocoClient.Move(vx, vy, wz)
              (STOP override)              │
                                     G1 walks
```

**Safety override:** when `/g1/human_cmd` is `STOP`, all Nav2 velocity commands are blocked regardless of what Nav2 requests. Robot only resumes when `NORMAL_OPERATION` is received.

**Stub mode:** if Unitree SDK is not installed or the G1 IP is unreachable, `G1Robot` prints commands to stdout instead of sending them — useful for development without hardware.

---

### `g1_bringup` — Launch Files and Configuration

| File | Description |
|---|---|
| `launch/hardware.launch.py` | **Full hardware stack** — single command for demo |
| `launch/genesis.launch.py` | Genesis simulation mode |
| `launch/offline.launch.py` | Rosbag offline pipeline test |
| `launch/sim.launch.py` | Gazebo Classic simulation (camera limited by driver issues) |
| `config/nav2_params.yaml` | Nav2 tuned for G1 footprint and walking speed |

---

## Three Execution Modes

### Mode 1 — Real Hardware

Runs the complete system on the real G1 with a mounted RealSense D435i.

```
RealSense D435i → RTAB-Map → /map
RealSense D435i → YOLO → Human detection
Nav2 → /cmd_vel → G1 SDK → G1 walks
```

**Prerequisites:**
- G1 powered on, connected via Ethernet or WiFi
- RealSense D435i mounted at `d435_link` position on G1 torso
- YOLO model file available (`yolo11n.pt` or `yolo26n.engine` on Jetson)

```bash
# Install Nav2
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-nav2-regulated-pure-pursuit-controller

# Launch everything
source /opt/ros/humble/setup.bash
source ~/Desktop/g1_slam/ros2_ws/install/setup.bash
ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161
```

**Arguments:**

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `192.168.123.161` | G1 IP address |
| `model_path` | `yolo11n.pt` | YOLO model (use `yolo26n.engine` on Jetson) |
| `rviz` | `true` | Launch RViz visualizer |
| `slam_mode` | `mapping` | `mapping` (build new map) or `localization` (use existing) |

**Launch sequence (automatic):**
- t=0s: RealSense + robot_state_publisher
- t=0s: SLAM starts
- t=8s: Human detection + obstacle injection start (after SLAM initialises)
- t=12s: Nav2 starts (after SLAM has a map)
- t=5s: G1 locomotion node connects to SDK

---

### Mode 2 — Genesis Simulation

Runs physics simulation with the G1 **MuJoCo model** (`genesis_sim/g1_29dof.xml`), two scripted human actors, and a virtual RGB-D camera. Publishes ROS2 topics that feed into the exact same SLAM, detection, and navigation stack as hardware.

The MuJoCo model is used instead of the URDF because it has exact foot contact geometry, correct joint definitions, and is the same format used to train the Unitree RL locomotion policy — giving much more accurate physics and better sim-to-real transfer.

```
Genesis physics + renderer → /camera/* topics → RTAB-Map + Detection
Genesis human ground truth → /humans/markers → Nav2 costmap
Nav2 /cmd_vel → Genesis → moves robot in simulation
```

**What is and isn't the same as hardware:**

| Component | Same as hardware? | Notes |
|---|---|---|
| SLAM (RTAB-Map) | ✅ Yes | Same code, same config |
| Nav2 planner | ✅ Yes | Same code, same config |
| Avoidance logic | ✅ Yes | Same thresholds, same override |
| Human detection (YOLO) | ❌ No | Genesis publishes ground truth positions directly — cylinders not detectable by YOLO |
| Low-level locomotion | ❌ No | Kinematic (slides) or RL policy — not Unitree's MPC at 1000Hz |

**Genesis cameras — two separate cameras:**

| Camera | Purpose | How configured |
|---|---|---|
| `scene.add_camera()` | Robot's D435 sensor — renders images published to `/camera/color/image_raw` for SLAM + detection | Positioned at exact D435 mount: `xyz=(0.0576, 0.0175, 1.267)`, pitch=47.6° |
| `viewer_options` in Scene | Spectator camera — what you see in the Genesis window with `--viewer` | `camera_pos=(5, -5, 4)` looking at origin |

**Known Genesis issues and fixes:**

| Issue | Fix |
|---|---|
| `numba` import error | `pip3 install "coverage>=7.0"` |
| `Scene.add_entity() got unexpected keyword argument 'pos'` | Genesis 1.0: pos goes inside morph |
| `camera.render() too many values to unpack` | Genesis 1.0 returns 4 values: `rgb, depth, _, _ = cam.render(...)` |
| Robot moves in reverse | Genesis quaternion is `(w, x, y, z)` not `(x, y, z, w)` |
| Simulation laggy with `--viewer` | Camera auto-reduces to 320×240 @ 5Hz in viewer mode |

**Install Genesis:**
```bash
pip3 install genesis-world
# Fix numba/coverage conflict if needed:
pip3 install "coverage>=7.0"
```

**Run (single command — opens 4-pane tmux session):**
```bash
# Kinematic mode — robot slides, no leg motion
./run_genesis.sh

# With GPU viewer window (RTX GPU required)
./run_genesis.sh --viewer

# With RL walking policy (realistic bipedal walking)
./run_genesis.sh --policy /path/to/g1_policy.pt

# With both
./run_genesis.sh --viewer --policy /path/to/g1_policy.pt

# Stop everything
./stop_genesis.sh
```

**Tmux pane layout:**
```
┌─────────────────────┬─────────────────────┐
│  Genesis bridge     │  ROS2 stack         │
│  (starts now)       │  (starts after 8s)  │
├─────────────────────┼─────────────────────┤
│  RViz               │  Teleop             │
│  (starts after 15s) │  (starts after 10s) │
└─────────────────────┴─────────────────────┘
Ctrl+B + arrow keys to switch panes
```

**Teleop keys:** `i` forward, `,` backward, `j` rotate left, `l` rotate right, `k` stop

**Running autonomous navigation + avoidance:**

After `./run_genesis.sh`, the Nav2 lifecycle manager can be flaky to autostart. If goals are
not accepted, manually activate Nav2:
```bash
./activate_nav2.sh
# confirm all three report "active [3]":
for n in controller_server planner_server bt_navigator; do
  echo -n "$n: "; ros2 lifecycle get /$n 2>/dev/null | grep -v XMLPARSER | head -1
done
```

Send a navigation goal (use the `odom` frame — see TF note below):
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
Or click **2D Goal Pose** in RViz.

The robot turns to face the goal, drives to it, and stops. When a simulated human
(red cylinder) comes within the front-stop distance, `/g1/human_cmd` publishes `STOP` and the
robot freezes until the human clears, then resumes — this is the avoidance demo.

**Genesis simulation notes / gotchas (learned the hard way):**

| Symptom | Cause | Fix (already applied) |
|---|---|---|
| Robot reaches 1st goal then all later goals fail | Two publishers on `map→odom` (static TF + RTAB-Map SLAM node) flicker once the pose graph drifts | `publish_tf: false` on the rtabmap SLAM node — static publisher owns `map→odom` |
| `follow_path` aborts continuously, no `/cmd_vel` | RPP collision detection + costmaps treating empty/unknown space as lethal | `use_collision_detection: false`, `track_unknown_space: false`, costmaps use inflation-only |
| Robot loops in a figure-8, never stops at goal | Strict `yaw_goal_tolerance` — arrives at position but wrong heading | `yaw_goal_tolerance: 3.15` (position-only goal checker) |
| Robot won't turn around for a goal behind it | `use_rotate_to_heading: false` + no reversing | `use_rotate_to_heading: true` with loose yaw tolerance |
| Camera only ~1 Hz, SLAM starves | CPU rendering 640×480 too slow | Camera at 160×120; larger RTAB-Map sync queue |
| Goals must be sent in the `odom` frame | Visual odom is feature-poor in the plain room, so `map→odom` is a static identity; `map` and `odom` coincide | Send goals in `odom` (or `map`, they're equal) |

> These tweaks make Nav2 robust in the *feature-poor kinematic simulation*. On real hardware
> (real RealSense, real features, real costmap obstacle data) the standard obstacle-aware
> settings apply — see `hardware.launch.py`.

**Download RL policy:**
```bash
git clone https://github.com/unitreerobotics/unitree_rl_gym
# Look for: logs/g1/policy.pt or runs/g1_*/policy.pt
```

**RL policy details:**

The `policy.py` module wraps Unitree's pre-trained RL locomotion policy:
- **Inputs (53 dims):** IMU angular velocity, gravity vector, cmd_vel, joint positions (15), joint velocities (15), previous actions (15), gait phase (4)
- **Outputs (15 dims):** joint position offsets for legs + waist (12 leg + 3 waist: yaw, roll, pitch)
- **Runs at:** 50Hz (policy) / 50Hz (physics)
- **Joint ordering:** matches MuJoCo actuator order in `g1_29dof.xml`
- **Falls back to:** PD standing controller if no checkpoint provided

When a checkpoint is loaded:
- Robot uses full articulated body physics
- Legs actually move and make contact with ground
- Walking behaviour matches real hardware (policy trained with domain randomisation)

---

### Mode 3 — Offline Rosbag Test

Validates the perception + planning pipeline using pre-recorded sensor data. No hardware or simulation required.

**What it tests:**
- SLAM builds a clean map ✅
- Human detection finds people ✅
- Human positions appear in Nav2 costmap ✅
- Nav2 plans paths around humans ✅
- `/cmd_vel` scales to zero when human too close ✅

**What it does not test:**
- Actual robot walking ❌

**Step 1 — Record a bag at the lab:**
```bash
ros2 bag record -o ~/lab_test \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info \
  /camera/depth/points \
  /imu_in_torso/data \
  /tf /tf_static
```
Walk around the room, then walk in front of the camera. 2-3 minutes is enough.

**Step 2 — Play back anywhere:**
```bash
cd ~/Desktop/g1_slam
./test_offline.sh ~/lab_test
```

Or manually:
```bash
# Terminal 1 — pipeline
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch g1_bringup offline.launch.py

# Terminal 2 — play bag
ros2 bag play ~/lab_test --clock --rate 1.0
```

**Verify avoidance is working:**
```bash
ros2 topic echo /g1/human_cmd    # STOP / SLOW_DOWN / NORMAL_OPERATION
ros2 topic echo /cmd_vel          # should be zero when human too close
```

---

## Setup

### 1. Install ROS2 Humble

```bash
cd ~/Desktop/g1_slam
chmod +x install_ros2.sh
./install_ros2.sh
source ~/.bashrc
```

### 2. Install Python dependencies

```bash
sudo pip3 install ultralytics numpy torch opencv-python lap roslibpy
pip3 install genesis-world              # for Genesis simulation only
pip3 install unitree_sdk2py             # for hardware only (or clone from GitHub)
```

### 3. Install Nav2

```bash
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-nav2-regulated-pure-pursuit-controller \
                 ros-humble-realsense2-camera \
                 ros-humble-rtabmap-ros
```

### 4. Build the workspace

```bash
cd ~/Desktop/g1_slam/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Add to `~/.bashrc` to avoid sourcing every time:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Desktop/g1_slam/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 5. Verify build

```bash
# Check all packages built
ros2 pkg list | grep g1

# Check detection node imports
python3 -c "from g1_detection.detection_node import DetectionNode; print('OK')"
```

---

## Hardware Requirements

| Component | Specification |
|---|---|
| Robot | Unitree G1 (29 DOF) |
| Camera | RealSense D435i (included with G1, mounted at `d435_link` on torso) |
| Compute | Laptop or NUC running Ubuntu 22.04 + ROS2 Humble |
| Network | G1 connected via Ethernet (static IP `192.168.123.161`) |
| GPU | Optional — YOLO runs on CPU but GPU recommended for latency |

**G1 sensors:**
- RealSense D435i — mounted on torso, frame defined as `d435_link` in URDF
- IMU in torso (`imu_in_torso`) — used by SLAM
- IMU in pelvis (`imu_in_pelvis`)
- Livox Mid360 LiDAR — frame defined as `mid360_link` in URDF (available for future use)

> **Note:** The MuJoCo model (`g1_29dof.xml`) only defines the mechanical structure (joints, actuators). It does not include sensor definitions — those are in the URDF. The RealSense is physically part of the G1 package.

**Supported cameras (if using a different camera):**

| Camera | Launch argument | Notes |
|---|---|---|
| RealSense D435i | `camera:=realsense` (default) | Built into G1 |
| ZED 2i | `camera:=zed` | Alternative, used in Human_dtp |

**For Jetson deployment (higher performance YOLO):**
- Use `yolo26n.engine` (TensorRT optimised, from the `Human_dtp` repo)
- Pass `model_path:=yolo26n.engine` to the launch file
- Jetson must be on the same network as G1

---

## RViz Visualisation

When running any mode, open RViz to see the full pipeline:

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=false
```

**Recommended displays:**

| Display | Topic | What it shows |
|---|---|---|
| Map | `/map` | SLAM occupancy grid |
| PointCloud2 | `/cloud_map` | 3D SLAM point cloud |
| RobotModel | `/robot_description` | G1 model |
| MarkerArray | `/humans/markers` | Red cylinders = humans, green lines = predicted paths |
| PointCloud2 | `/human_obstacle_cloud` | Human positions in Nav2 costmap |
| Path | `/plan` | Nav2 planned path |

Set **Fixed Frame** to `map`.

---

## Key Topics Reference

| Topic | Type | Published by | Description |
|---|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RealSense / Genesis | RGB camera feed |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RealSense / Genesis | Depth aligned to RGB |
| `/imu_in_torso/data` | `sensor_msgs/Imu` | RealSense / Genesis | IMU for SLAM |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map | 2D occupancy map |
| `/odom/raw` | `nav_msgs/Odometry` | Genesis / RTAB-Map | Robot odometry |
| `/humans/markers` | `visualization_msgs/MarkerArray` | detection_node / Genesis | Human positions + predictions |
| `/g1/human_cmd` | `std_msgs/String` | detection_node / Genesis | STOP / SLOW_DOWN / NORMAL_OPERATION |
| `/human_obstacle_cloud` | `sensor_msgs/PointCloud` | human_obstacle_node | Humans as Nav2 obstacles |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 | Velocity commands to robot |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz (2D Nav Goal) | Navigation target |

---

## Relationship to Human_dtp

This repo builds on [`Human_dtp`](../Human_dtp), which was originally built for a **MiR AMR (wheeled robot)**. The detection pipeline (`detection.py`, `eigen.py`, `kf.py`, `preprocessing.py`, `EigenTrajectory/`) is directly ported with only import path changes. The key difference:

| Human_dtp | g1_slam |
|---|---|
| MiR AMR (wheeled) | Unitree G1 (humanoid) |
| ZED 2i camera | RealSense D435i |
| ZMQ transport | ROS2 topics |
| Docker on Jetson | ROS2 workspace |
| REST API robot control | Unitree SDK |
| No SLAM (assumes fixed route) | Full RTAB-Map SLAM |
| No Nav2 | Nav2 autonomous navigation |

---

## Implementation Plan

See [`PLAN.md`](PLAN.md) for the full 3-week step-by-step plan including:
- Exact code to write for each step
- Tuning parameters
- Risk register
- Single launch command reference

---

## Troubleshooting

**gzserver crashes on launch:**
NVIDIA driver 595+ has a rendering incompatibility with Gazebo Classic. Use Genesis or offline rosbag instead.

**`numpy.core.multiarray failed to import`:**
System matplotlib conflicts with NumPy 2.x. Fix:
```bash
sudo pip3 install --upgrade matplotlib
```

**Detection node crashes on import:**
EigenTrajectory import fails — this is handled gracefully and EigenTrajectory is disabled by default. If the error persists, check that `g1_detection/eigen.py` has the try/except around the import.

**G1 SDK connection fails:**
```bash
# Check G1 is reachable
ping 192.168.123.161
# Check SDK version matches firmware
python3 -c "import unitree_sdk2py; print(unitree_sdk2py.__version__)"
```

**RTAB-Map not receiving data:**
```bash
ros2 topic hz /camera/color/image_raw    # should be ~30 Hz
ros2 topic hz /imu_in_torso/data         # should be ~200 Hz
```

**Nav2 not planning paths:**
```bash
ros2 topic echo /map --once              # confirm map is being published
ros2 run tf2_tools view_frames && evince frames.pdf   # check TF tree
```
