# End-to-end ZED + RTAB-Map + Nav2 + Human-Detection Test

> One-command launcher + step-by-step recovery for the full perception →
> planning → velocity loop using the **ZED 2i**, **RTAB-Map**, **YOLO human
> detection**, and **Nav2** — no robot required. RViz is the viewer.
> Pairs with [ZED2_TEST.md](ZED2_TEST.md) (standalone detection) and
> [README.md](README.md) (full hardware architecture).

## 0. What this validates

```
ZED 2i ──► zed_wrapper ──► /zed/zed_node/{rgb,depth,point_cloud,camera_info,imu}
            │
            ├──► RTAB-Map ──► /map, map→odom TF
            │
            ├──► detection_node ──► /humans/markers, /g1/human_cmd, /human_obstacle_cloud
            │
            └──► Nav2 (rolling global+local costmap on /odom) ──► /cmd_vel
                                                                    │
robot_state_publisher (G1 URDF) ──► /robot_description ──► RViz RobotModel
static TF zed_camera_link → pelvis
```

Success criteria:
- RViz renders the live ZED point cloud, the G1 URDF, and human cylinders.
- Sending a 2D goal makes `/cmd_vel` publish at ~20 Hz.
- Walking in front of the camera flips `/g1/human_cmd` between
  `NORMAL_OPERATION` / `SLOW_DOWN` / `STOP`.

## Staged bring-up — test each part in order (DO THIS FIRST)

Bring the pipeline up one layer at a time. Each stage has a single command to
run and a clear pass check. **Do not skip ahead** — a failure is far easier to
find in its own stage than inside the full stack. 🟢 = no robot motion (safe),
🔴 = the robot can move (clear space, e-stop in hand, robot already STANDING).

Every terminal first:
```bash
cd ~/Desktop/g1_slam && source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
NIC=$(ip -br addr | awk '/192\.168\.123\./{print $1}')   # host iface on the robot subnet
echo "robot NIC = ${NIC:-NOT FOUND — is the G1 connected?}"
```

### Stage 0 — Build 🟢
```bash
cd ~/Desktop/g1_slam/ros2_ws && colcon build --packages-select g1_detection g1_bringup g1_robot && source install/setup.bash
```
**Pass:** build finishes with no errors.

### Stage 1 — Locomotion wrapper (no camera) 🔴
Validates `robot_node → g1.py → Move()`. Stand the robot first (remote, or the
`g1_loco_client_example` StandUp).
```bash
# terminal A
ros2 run g1_robot robot_node --ros-args -p net_iface:=$NIC
# terminal B — make it take one step, then stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}}"
```
**Pass:** terminal A logs `Connected via DDS` (NOT `[G1 STUB]`) and the robot takes a real step on the first publish.
**If it fails:** `[G1 STUB]` = SDK import/connection problem; no step but not stub = robot not in a walking-ready stance (re-stand it).

### Stage 2 — Blind autonomous nav (no camera) 🔴
Validates `odom_bridge` + Nav2 + `robot_node` together. Goals in the **`map`** frame.
```bash
# terminal A
ros2 launch g1_bringup nav_tier3.launch.py net_iface:=$NIC
# terminal B — check odom + TF before sending a goal
ros2 run tf2_ros tf2_echo map pelvis          # should print a moving transform
#   (watch terminal A for "Odom source = sportmodestate | imu+cmd_vel | cmd_vel")
# terminal B — short goal, 1 m ahead
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```
**Pass:** robot walks ~1 m forward and stops near the goal.
**If it fails:** no `/cmd_vel` → Nav2 lifecycle didn't activate (watch the `manage_nodes` call ~12 s in); walks but never stops → odom drift, check the `Odom source` log.

### Stage 3 — Camera stream (no robot) 🟢
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
# new terminal
ros2 topic hz /zed/zed_node/rgb/color/rect/image
ros2 topic hz /zed/zed_node/depth/depth_registered
```
**Pass:** both report a steady ~15–30 Hz. (Full setup/troubleshooting: [ZED2_TEST.md](ZED2_TEST.md).)

### Stage 4 — Human detection (no robot) 🟢
Validates YOLO → `/g1/human_cmd` against the live ZED (with Stage 3 still running).
```bash
ros2 run g1_detection detection_node --ros-args \
  -p model_path:=yolo26n.pt \
  -p color_topic:=/zed/zed_node/rgb/color/rect/image \
  -p depth_topic:=/zed/zed_node/depth/depth_registered \
  -p camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info \
  -p camera_frame:=zed_left_camera_frame
# new terminal
ros2 topic echo /g1/human_cmd
```
**Pass:** the command flips `NORMAL_OPERATION → SLOW_DOWN → STOP` as you walk toward the camera (thresholds in [ZED2_TEST.md](ZED2_TEST.md) §4).

### Stage 5 — Camera localization + TF (no robot) 🟢
Brings up the full perception stack, no motion. Pick your odom source:
```bash
USE_ZED_ODOM=1 ./run_zed_e2e.sh      # ZED VIO + area memory   (or omit for RTAB-Map)
# in a sourced shell:
ros2 run tf2_tools view_frames        # opens frames.pdf
ros2 run tf2_ros tf2_echo map pelvis  # should be stable, not jumping wildly
ros2 topic echo /rtabmap/odom         # odom updates as you move the camera
```
**Pass:** `view_frames` shows **one** tree `map → odom → … → pelvis` (no frame with two parents), and `map→pelvis` is stable.
**If it fails:** two `map→odom` publishers or a broken chain → this is the #1 cause of "robot won't navigate" later. Fix here before Stage 6.

### Stage 6 — Nav2 produces velocity + human cloud (no robot) 🟢
Same stack as Stage 5. Goals in the **`odom`** frame (rolling costmap).
```bash
ros2 topic hz /cmd_vel                 # start watching
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
ros2 topic echo /human_obstacle_cloud --once   # PointCloud2 when a person is in view
```
**Pass:** `/cmd_vel` publishes ~20 Hz toward the goal; `/human_obstacle_cloud` is a `PointCloud2` when someone is present. (Costmap injection of that cloud stays disabled — see §7.)

### Stage 7 — Full end-to-end (camera + robot) 🔴
Everything together. Short goal, clear space, e-stop ready, robot STANDING.
```bash
WITH_ROBOT=1 NET_IFACE=$NIC USE_ZED_ODOM=1 ./run_zed_e2e.sh
# in a sourced shell — short goal in 'odom':
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
# watch: ros2 topic echo /g1/human_cmd     ros2 topic hz /cmd_vel
```
**Pass:** robot walks toward the goal and **halts when you step into its path, resumes when you clear** (stop-for-humans). Reaching the goal cleanly depends on how well ZED VIO tracks the walking robot — that's the part you tune.

## 1. One-shot launcher (recommended)

```bash
cd ~/Desktop/g1_slam
./run_zed_e2e.sh
```

That spins up an attached `tmux` session `g1_zed` with all 8 components in
two windows of split panes:

| Window  | Pane | Component                       |
|---------|------|---------------------------------|
| core    | 0    | `zed_wrapper`                   |
| core    | 1    | `rtabmap_launch`                |
| core    | 2    | `g1_detection/detection_node`   |
| core    | 3    | `nav2_bringup` + lifecycle activate |
| extras  | 0    | static TF `zed_camera_link → pelvis` |
| extras  | 1    | `robot_state_publisher` (G1 URDF) |
| extras  | 2    | `rviz2`                         |
| extras  | 3    | shell for `ros2 topic ...` / goal commands |

tmux basics:
- Switch panes: `Ctrl-b` then arrow keys
- Switch windows: `Ctrl-b n` / `Ctrl-b p`
- Detach: `Ctrl-b d`  → reattach with `tmux attach -t g1_zed`
- Kill everything: `./run_zed_e2e.sh stop`

Startup waits (~16 s) are baked in so each layer sees the topics it needs.

## 2. Send a goal and watch the loop

In `extras` window, pane 3 (monitor shell) — or any sourced shell:

```bash
# Watch velocity
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel --field linear.x

# Watch the safety signal
ros2 topic echo /g1/human_cmd

# Send a goal (frame_id MUST be 'odom' — global costmap is odom-based)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'odom'},
           pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback

# Cancel
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --cancel
```

Or in RViz: toolbar → **2D Goal Pose** → click 2–3 m ahead.

Walk into the camera FoV:
- ~3 m → `/g1/human_cmd` = `SLOW_DOWN` (robot_node would scale velocity ×0.4)
- ~1.5 m → `STOP` (robot_node would zero `/cmd_vel`)
- Step out → `NORMAL_OPERATION` resumes

## 3. RViz setup (first time only)

The launcher opens RViz with the `nav2_default_view.rviz` preset. Tweak it:

1. **Global Options → Fixed Frame** = `map`
2. **Views panel → Type** = `Orbit` (default `TopDownOrtho` only gives top-down)
3. Add displays:
   - PointCloud2 → `/zed/zed_node/point_cloud/cloud_registered`
   - Image       → `/zed/zed_node/rgb/color/rect/image`
   - MarkerArray → `/humans/markers`
   - RobotModel  → Description Topic `/robot_description`
4. Save: **File → Save Config As** (e.g. `~/Desktop/g1_slam/zed_e2e.rviz`).
   Next launch: `rviz2 -d ~/Desktop/g1_slam/zed_e2e.rviz` (or edit the
   launcher to point at this file).

## 4. Manual launch (7-terminal fallback)

If you prefer/need to run components individually, every terminal first:

```bash
cd ~/Desktop/g1_slam
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/targets/x86_64-linux/lib:$LD_LIBRARY_PATH
```

| # | Command |
|---|---------|
| 1 | `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i pos_tracking.pos_tracking_enabled:=false` |
| 2 | `ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id zed_camera_link --child-frame-id pelvis` |
| 3 | `ros2 launch rtabmap_launch rtabmap.launch.py rgb_topic:=/zed/zed_node/rgb/color/rect/image depth_topic:=/zed/zed_node/depth/depth_registered camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info frame_id:=zed_camera_link approx_sync:=true approx_sync_max_interval:=0.02 qos:=2 rtabmap_viz:=false rviz:=false` |
| 4 | `ros2 run g1_detection detection_node --ros-args -p model_path:=yolo26n.pt -p color_topic:=/zed/zed_node/rgb/color/rect/image -p depth_topic:=/zed/zed_node/depth/depth_registered -p camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info -p camera_frame:=zed_left_camera_frame` |
| 5 | `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=$(ros2 pkg prefix g1_bringup)/share/g1_bringup/config/nav2_params_hw.yaml` then `ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 2}"` |
| 6 | `URDF=$(ros2 pkg prefix g1_description)/share/g1_description/urdf/g1_29dof.urdf; ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat $URDF)"` |
| 7 | `rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz` |

## 5. Verification commands

```bash
# Topics
ros2 topic hz /zed/zed_node/rgb/color/rect/image     # ~15–30 Hz
ros2 topic hz /zed/zed_node/depth/depth_registered   # ~15–30 Hz
ros2 topic hz /rtabmap/odom                          # ~15–30 Hz
ros2 topic hz /map                                   # ~1 Hz
ros2 topic hz /humans/markers                        # only while a human is seen
ros2 topic hz /cmd_vel                               # only while a goal is active

# TF
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map pelvis
ros2 run tf2_ros tf2_echo zed_camera_link zed_left_camera_frame_optical

# Nav2 lifecycle
ros2 lifecycle get /bt_navigator           # active
ros2 lifecycle get /controller_server      # active
ros2 lifecycle get /planner_server         # active

# Plot velocity
ros2 run rqt_plot rqt_plot /cmd_vel/linear/x /cmd_vel/linear/y /cmd_vel/angular/z
```

## 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Nav2 fails to activate with `clear_entirely_global_costmap not available` + `Exception when loading BT` | Global/local costmap failed to initialize. Most common: an `observation_sources` entry with `data_type: "PointCloud"` (Nav2 only supports `PointCloud2`/`LaserScan`). Check [nav2_params_hw.yaml](ros2_ws/src/g1_bringup/config/nav2_params_hw.yaml). |
| `Node not recognized: <X>` in BT XML | Add `nav2_<X>_bt_node` (snake_case) to `bt_navigator.plugin_lib_names` in [nav2_params_hw.yaml](ros2_ws/src/g1_bringup/config/nav2_params_hw.yaml). |
| `Robot is out of bounds of the costmap!` | Global costmap is empty or static-layer-only. Stack is configured as rolling on `odom` (20×20 m) — make sure that didn't get reverted. |
| Goal returns `distance_remaining: 0.0` immediately | Planner produced no path (empty costmap), or goal is within `xy_goal_tolerance`. Move the camera ~15 s to seed the local cloud, send a goal further away (3+ m). |
| `extrapolation` warnings from `rgbd_odometry` looking up `pelvis → zed_left_camera_frame_optical` | RTAB-Map was launched with `frame_id:=pelvis`. Use `frame_id:=zed_camera_link` (see manual command #3). |
| `Could not resolve host: meshes` from RViz | URDF used was the bare-paths copy at the repo root. Use the packaged URDF: `$(ros2 pkg prefix g1_description)/share/g1_description/urdf/g1_29dof.urdf`. |
| `package 'zed_wrapper' not found` | Terminal didn't `source ros2_ws/install/setup.bash`. |
| `Unable to load library: libnvinfer_builder_resource.so.10.13.2` | Loader path missing CUDA 13 TRT lib. `export LD_LIBRARY_PATH=/usr/local/cuda-13.0/targets/x86_64-linux/lib:$LD_LIBRARY_PATH`. See [ZED2_TEST.md §6.2](ZED2_TEST.md). |
| `CORRUPTED SDK INSTALLATION` / `NEURAL CORRUPTED MODEL` | Re-optimize: `sudo /usr/local/zed/tools/ZED_Diagnostic -aio && sudo /usr/local/zed/tools/ZED_Diagnostic -nrlo_all`. |
| `Camera model does not match user parameter 'zed2i'` | You launched with `camera_model:=zed2`; use `zed2i`. |
| `Gravity alignment issues detected` | IMU re-aligning. Hold camera still ~2 s at startup. |
| Two markers per human in RViz | By design: red cylinder (`ns=humans`) + green prediction line (`ns=human_predictions`). Toggle namespaces in the MarkerArray display. |

## 7. What this test does NOT validate

- **Real G1 locomotion** — `/cmd_vel` is produced but no leg motion happens.
  To exercise the SDK, start `g1_robot/robot_node` (stub mode prints velocities;
  on the real robot with reachable IP it walks). See [README.md](README.md).
- **Persistent global map** — global costmap is rolling on `odom`. If you need
  the static-map planning path used on hardware, set `global_costmap.global_frame: map`
  and add `static_layer` back to the plugins list (and ensure RTAB-Map's `/map`
  has accumulated enough cells before sending goals).
- **Humans as costmap obstacles** — `/human_obstacle_cloud` is published but
  Nav2 only consumes `PointCloud2`. Convert the source (or republish as
  `PointCloud2`) to re-enable obstacle injection. Avoidance via
  `/g1/human_cmd` STILL works.
