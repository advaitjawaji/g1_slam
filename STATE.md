# Repository State — g1_slam

> Agent-oriented status document. Read this first to get full context.
> Last updated: 2026-06-04. Pairs with `README.md` (usage) and `PLAN.md` (roadmap).

## 1. What this project is

Autonomous navigation + human avoidance for the **Unitree G1 humanoid**, built on
ROS 2 Humble. The G1 maps its environment (SLAM), detects humans, and uses Nav2 to
plan collision-free paths, stopping/rerouting around people. The detection pipeline is
ported from a working sibling repo, `Human_dtp` (a MiR-AMR system deployed on Jetson).

Three run modes: **Genesis simulation**, **offline rosbag replay**, **real hardware**.

## 2. Current status — what is and isn't validated

### Validated in Genesis simulation ✅
- RTAB-Map RGB-D SLAM builds a usable map.
- Nav2 autonomous navigation: accepts goals, plans, follows path, **reaches goal and stops**, turns around for goals behind it. Sequential multi-goal works.
- Human markers appear; reactive avoidance **stops the robot** when a human is in the front danger zone and resumes when clear.
- Full ROS 2 graph wiring, frames, topic compatibility.

### Coded but NOT yet validated (first run will be on hardware) ❌
- **`detection_node` / `HumanXZPredictor` (real YOLO).** In sim this was **bypassed** — `genesis_node` published ground-truth human positions and computed the stop command with its own inline thresholds. The real detection code has **never run in this repo's loop**. (It is, however, proven in `Human_dtp` on real hardware.)
- **`robot_node` + `g1.py` (Unitree SDK).** Never executed. The `G1LocoClient` import path / method names (`Move`, `BalanceStand`) are from docs and are **unverified against real SDK version + G1 firmware**. Highest-risk unknown.
- **Obstacle-aware costmaps** (`nav2_params_hw.yaml`). The depth-cloud obstacle layers were OFF in sim; untested.
- **ZED 2 camera path.** Detection code is ZED-ready (see §6) but the `zed-ros2-wrapper` node/topic config is unverified against the installed version.

### Key sim-vs-hardware divergence (important)
The sim validated the **integration + navigation framework**, NOT the **perception or robot-control code**. See §5.

## 3. Architecture / data flow

```
Waypoint (RViz 2D Goal Pose / ros2 action send_goal / demo_tour.sh)
   → Nav2 (bt_navigator → planner_server → controller_server)
   → /cmd_vel (Twist: vx, vy, wz)
   → [SIM] genesis_node moves fake body   |   [HW] robot_node → g1.py → Unitree SDK Move() → real walk

Camera (sim render / RealSense / ZED)
   → /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw, /camera/color/camera_info
   → RTAB-Map (map + odom)        → /map, TF map→odom
   → detection_node (YOLO+depth)  → /humans/markers, /g1/human_cmd
   → human_obstacle_node          → /human_obstacle_cloud (Nav2 obstacle, HW only)
```
- `g1.py` receives **velocities only**, never waypoints. Nav2 does all waypoint→path→velocity conversion. The Unitree high-level controller is a velocity interface (`Move(vx,vy,vyaw)`).
- The Unitree SDK is a **client library** on the host; the actual walking controller is **proprietary firmware on the G1's onboard computer**. It cannot run in sim.

## 4. Package / file map

```
ros2_ws/src/
├── g1_description/    URDF (g1_29dof.urdf), meshes, Gazebo world. Sensors in URDF:
│                      d435_link (RealSense), mid360_link (Livox LiDAR), imu_in_torso.
├── g1_slam/           RTAB-Map config (config/rtabmap.yaml) + slam/localization launch.
├── g1_detection/      detection.py (HumanXZPredictor, from Human_dtp),
│                      detection_node.py (ROS2 wrapper, camera-agnostic),
│                      human_obstacle_node.py, eigen.py, kf.py, EigenTrajectory/,
│                      overlay_render.py (pure cv2/numpy renderer, no ROS import),
│                      overlay_node.py (demo video: annotated egocentric MP4).
├── g1_robot/          robot_interface.py, g1.py (Unitree SDK — UNVERIFIED),
│                      robot_node.py (/cmd_vel + /g1/human_cmd → SDK).
└── g1_bringup/        launch/{genesis,offline,hardware,sim}.launch.py,
                       config/{nav2_params_sim.yaml, nav2_params_hw.yaml, rtabmap...}.

genesis_sim/           genesis_node.py (sim bridge: camera render + body + avoidance override),
                       policy.py (RL locomotion wrapper — needs a checkpoint to walk),
                       g1_29dof.xml (MuJoCo model), meshes -> symlink.

tools/                 Off-robot validation (no ROS needed):
  preview_overlay.py                 synthetic scene -> overlay MP4 + stills
  test_overlay_geometry.py           hand-derived pinhole/clipping checks
  test_detection_world_track.py      world-track correctness + camera-frame regression

Top-level scripts:
  run_genesis.sh / stop_genesis.sh   tmux launcher for Genesis sim (--viewer, --policy)
  activate_nav2.sh                   manually activate Nav2 lifecycle (it is flaky on autostart)
  demo_tour.sh                       hands-free multi-goal tour for demo recording
  test_offline.sh                    rosbag replay pipeline test
  install_ros2.sh                    ROS2 Humble + deps installer
```

## 5. Nav2 params are split (do not cross-contaminate)
- `nav2_params_sim.yaml` — **permissive** for the feature-poor sim: collision detection OFF, costmaps inflation-only, position-only goal checker. Used by `genesis.launch.py` and `offline.launch.py`.
- `nav2_params_hw.yaml` — **obstacle-aware**: depth-cloud obstacle layers, static SLAM map layer, collision detection ON. Used by `hardware.launch.py`.

## 5b. Demo overlay (annotated egocentric video) — see `DEMO_VIDEO.md`

`overlay_node` renders the ZED view with Human_dtp bounding boxes, each person's
forecast trajectory, the Nav2 plan and the path actually walked, plus a
bird's-eye inset; publishes `/g1/overlay/image` and writes an MP4. On by default
in `run_zed_e2e.sh` (window `demo`) and `hardware.launch.py` (`overlay:=true`).

Two things worth knowing before touching it:
- It consumes `/g1/detections` (new JSON topic from `detection_node`) instead of
  running YOLO twice, and draws each result on the frame whose stamp it came
  from, so boxes don't lag inference.
- `detection.py` now keeps a second, **geometrically correct** per-person track
  (`tracks_world`, odom X-Y via a full 4x4 transform of the camera *optical*
  frame) purely for this overlay. The legacy `tracks_xz` path that `/g1/human_cmd`
  is tuned against is unchanged — verified bit-identical by
  `tools/test_detection_world_track.py`. See §8 for why the legacy one is wrong.

**Stop with `./run_zed_e2e.sh stop`**, not Ctrl-C on the terminal — the MP4 is
finalised on node shutdown.

## 6. Camera support
`detection_node` is camera-agnostic with configurable topic params:
`color_topic`, `depth_topic`, `camera_info_topic`, `camera_frame`.
- **RealSense D435i**: color `rgb8/bgr8`, depth `16UC1` (mm). Built into the G1 (`d435_link`).
- **ZED 2**: color `bgra8` (alpha dropped), depth `32FC1` (metres). Handled.
- Depth → full XYZ cloud via pinhole back-projection using `camera_info` intrinsics
  (needed so `HumanXZPredictor` reads each human's lateral X and forward Z).

Standalone detection test (no robot, no SLAM):
```bash
# RealSense
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true align_depth.enable:=true
ros2 run g1_detection detection_node --ros-args -p model_path:=yolo26n.engine
# ZED (verify topic names with: ros2 topic list | grep zed)
ros2 run g1_detection detection_node --ros-args \
  -p color_topic:=/zed/zed_node/rgb/image_rect_color \
  -p depth_topic:=/zed/zed_node/depth/depth_registered \
  -p camera_info_topic:=/zed/zed_node/rgb/camera_info \
  -p camera_frame:=zed_left_camera_frame
# watch: ros2 topic echo /g1/human_cmd   (STOP/SLOW_DOWN/NORMAL_OPERATION)
```

## 7. How to run

**Genesis sim (validated):**
```bash
./run_genesis.sh            # or --viewer for GPU window, --policy <pt> for RL walking
# wait ~35s, then:
./activate_nav2.sh          # Nav2 lifecycle autostart is unreliable; this forces it
# send goals in the `odom` frame (map==odom in sim) via RViz 2D Goal Pose or:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
./demo_tour.sh              # hands-free demo
```

**Hardware (untested):**
```bash
ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161   # camera:=zed optional
# goals in the `map` frame (real SLAM map)
```

## 8. Known gotchas (learned during debugging — all fixed in repo)
- **Nav2 "only first goal works"** → was duplicate `map→odom` publishers (static TF + RTAB-Map). Fixed with `publish_tf:false` on rtabmap SLAM node (sim).
- **`follow_path` aborts / no cmd_vel** → RPP collision detection + unknown-as-lethal costmap. Fixed in sim params (collision off, `track_unknown_space:false`).
- **Figure-8 looping, never stops** → strict yaw goal tolerance. Fixed: `yaw_goal_tolerance:3.15`.
- **Won't turn around for goals behind** → needed `use_rotate_to_heading:true` (works once yaw tol loose).
- **Genesis camera ~1Hz / SLAM starves** → CPU render; dropped to 160×120.
- **Genesis 1.0 API**: pos inside morph; `camera.render()` returns 4 values; quaternion (w,x,y,z).
- **numba import error** → `pip3 install "coverage>=7.0"`.
- **NumPy 2 vs cv_bridge/matplotlib** → removed cv_bridge dep (direct numpy image conversion), upgraded matplotlib.
- **colcon build `canonicalize_version`** → `pip3 install -U packaging`.
- **Human world positions skew once the robot walks** → `_camera_xz_to_odom_xz`
  in `detection.py` drops the optical Y axis and reads world rows 0+2, i.e. it
  treats world *height* as a ground axis and applies a body-frame transform to
  optical-frame points. Invisible in sim (TF ≈ identity, camera level). NOT fixed
  — the avoidance thresholds are tuned against it and `/human_obstacle_cloud` is
  commented out of `nav2_params_hw.yaml` because of it. The overlay sidesteps it
  via `tracks_world`; fix that transform before enabling costmap injection.
- **OpenCV Hershey fonts are ASCII-only** → any `·`, `—`, `°` in overlay text
  renders as `?`. `overlay_render.ascii_safe()` normalises; keep `title` ASCII.
- **zed_components "Could NOT find CUDA: required is at least 13"** → ZED SDK 5.2.3's cmake demands the CUDA 13 toolkit but the dev machine only has 12.5 (the SDK binary itself only needs the driver). Fixed: patched config at `ros2_ws/cmake/zed/`; build with `colcon build --packages-up-to zed_wrapper --cmake-args -DZED_DIR=$PWD/cmake/zed`. The wrapper clone (`ros2_ws/src/zed-ros2-wrapper`, v5.3.1) is gitignored — re-clone with `--recurse-submodules` on a fresh checkout. Smoke-tested 2026-06-10: node loads, topics are `/zed/zed_node/...`.

## 9. Immediate next steps (priority order)
1. **Detection test with real camera** (RealSense or ZED) — §6. Closes the biggest gap (real YOLO). No robot needed.
2. **Verify Unitree SDK API** on the real G1: `python3 -c "import unitree_sdk2py.g1.loco.g1_loco_client as m; print(dir(m))"` — reconcile with `g1.py`.
3. **G1 walks via SDK**: `ros2 topic pub /cmd_vel ...` → confirm a step.
4. **SLAM with real RealSense/ZED** in a feature-rich room (sim room was featureless).
5. **Full stack** via `hardware.launch.py`.

## 10. Sibling projects (context, separate repos)
- **Human_dtp** (`~/Desktop/Human_dtp`): origin of the detection pipeline; MiR AMR + ZED 2i, deployed/working.
- **RobustBench**: 5-week project — collect real G1 motor temp/effort data → perturbation functions → robust walking policy benchmark. Provides real thermal-derated torque limits.
- **Off-axis torque compensation** ("Extensions of RobustBench"): whole-body QP controller that recruits off-axis joints when a joint saturates. Separate new repo; can start sim-first using the torque limits already in `g1_29dof.xml` (`actuatorfrcrange`).
