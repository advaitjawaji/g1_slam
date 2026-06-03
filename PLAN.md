# G1 Human Avoidance Demo — Implementation Plan

## Goal
Unitree G1 humanoid robot that autonomously navigates a lab environment while detecting and avoiding humans in real time.

## System Architecture

```
RealSense D435i (on G1)
        │
        ├──► RTAB-Map ──────────────────► /map + /tf (odom → map)
        │
        └──► Detection Node ────────────► Human positions in world frame
                                               │
                                  ┌────────────┴────────────┐
                                  ▼                         ▼
                        Costmap Injection         Velocity Safety Override
                        (Nav2 routes around)      (hard stop if too close)
                                  │                         │
                                  └────────── Nav2 ─────────┘
                                                │
                                           /cmd_vel
                                                │
                                        G1 Loco Node
                                                │
                                        Unitree SDK
                                                │
                                           G1 walks
```

## Repos
- `~/Desktop/Human_dtp` — original human detection pipeline (MiR AMR)
- `~/Desktop/g1_slam` — G1 ROS2 workspace (SLAM + detection + navigation)

## ROS2 Packages (in g1_slam/ros2_ws/src/)
| Package | Purpose | Status |
|---|---|---|
| `g1_description` | G1 URDF + meshes + Gazebo world | ✅ Done |
| `g1_slam` | RTAB-Map config + launch files | ✅ Done |
| `g1_detection` | Human detection ROS2 node (ported from Human_dtp) | ✅ Done (numpy blocked) |
| `g1_robot` | Unitree G1 SDK interface + ROS2 node | ✅ Stubbed |
| `g1_bringup` | Top-level sim + hardware launch files | ✅ Done |

## Key Files
- `g1_detection/g1_detection/detection_node.py` — ROS2 node wrapping HumanXZPredictor
- `g1_detection/g1_detection/detection.py` — YOLO + depth + tracking (from Human_dtp)
- `g1_robot/g1_robot/g1.py` — Unitree SDK stub (needs completing)
- `g1_robot/g1_robot/robot_node.py` — ROS2 node: /cmd_vel → G1 SDK
- `g1_slam/config/rtabmap.yaml` — RTAB-Map parameters
- `g1_bringup/launch/hardware.launch.py` — full hardware stack launch
- `g1_bringup/launch/sim.launch.py` — Gazebo simulation launch

## Hardware
- **Robot:** Unitree G1 (29 DOF, in lab)
- **Camera:** RealSense D435i (mount on G1 torso, matches `d435_link` in URDF)
- **Compute:** Jetson (for YOLO inference) + laptop (for SLAM + Nav2)
- **Network:** G1 IP default `192.168.123.161`, configure in `hardware.launch.py`

## Known Issues / Blockers
| Issue | Fix |
|---|---|
| NumPy 2.x vs cv_bridge (ROS2 Humble) | Rebuild cv_bridge from source against NumPy 2.x |
| Gazebo Classic camera sensor crashes with NVIDIA 595 driver | Use real RealSense for hardware; Gazebo sim deferred |
| G1 SDK locomotion stub | Complete `g1_robot/g1.py` with `LocoClient` |
| Nav2 not yet configured | Write `nav2_params.yaml` |
| Human costmap injection not implemented | Write new ROS2 node |

---

## Phase 1 — Foundation (Days 1–3)

### Step 1: Fix NumPy / cv_bridge conflict (Day 1)
**Problem:** `cv_bridge` in ROS2 Humble was compiled against NumPy 1.x. `ultralytics` requires NumPy ≥ 2. The detection node crashes on import.

**Fix:** Rebuild cv_bridge from source against the installed NumPy 2.x.
```bash
cd ~/ros2_cv_bridge_ws/src
git clone https://github.com/ros-perception/vision_opencv.git -b humble
cd ..
colcon build --packages-select cv_bridge \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```
Then add `source ~/ros2_cv_bridge_ws/install/setup.bash` to `~/.bashrc` before the g1_slam workspace.

**Verify:**
```bash
python3 -c "from cv_bridge import CvBridge; print('OK')"
ros2 run g1_detection detection_node  # should not crash
```

### Step 2: Install Unitree SDK (Day 1)
```bash
pip3 install unitree_sdk2py
```
**Verify connection to G1:**
```bash
# G1 default IP: 192.168.123.161
# Connect laptop to G1's WiFi or ethernet
python3 -c "
from unitree_sdk2py.core.channel import ChannelFactory
ChannelFactory.Instance().Init(0, '192.168.123.161')
print('SDK connected')
"
```
Check G1 firmware version — SDK version must match.

### Step 3: Validate SLAM with RealSense (Days 2–3)
Mount RealSense on G1 (or hold it), run SLAM:
```bash
ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161
```
Walk around the lab carrying the camera. In RViz:
- Fixed Frame: `map`
- Add: Map → `/map`
- Add: PointCloud2 → `/cloud_map`

**Tune `g1_slam/config/rtabmap.yaml` if map is noisy:**
- Increase `Vis/MinInliers` (15 → 20) for stricter loop closure
- Decrease `Rtabmap/DetectionRate` (1.0 → 0.5) if CPU is overloaded
- Decrease `Grid/CellSize` (0.05 → 0.03) for finer map resolution

**Deliverable:** Clean 2D occupancy map of the lab saved to `rtabmap.db`.

---

## Phase 2 — G1 Locomotion + Navigation (Days 4–8)

### Step 4: Complete G1 locomotion node (Days 4–5)
File: `g1_robot/g1_robot/g1.py`

```python
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

class G1Robot(RobotInterface):
    def setup(self, config):
        ChannelFactory.Instance().Init(0, config.ip)
        self._client = LocoClient()
        self._client.SetTimeout(10.0)
        self._client.Init()
        self._client.Start()  # enter normal standing mode

    def send_command(self, cmd):
        if cmd.mode == RobotMode.STOP:
            self._client.StopMove()
        else:
            self._client.Move(cmd.vx, cmd.vy, cmd.wz)

    def stop(self):
        self._client.StopMove()
```

**Test sequence:**
```bash
# Terminal 1
ros2 launch g1_robot robot.launch.py robot_ip:=192.168.123.161

# Terminal 2 — send a slow forward command
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```
G1 should take a few steps forward.

### Step 5: Configure Nav2 for G1 (Days 6–7)
File: `g1_bringup/config/nav2_params.yaml`

Key parameters to set:
```yaml
bt_navigator:
  robot_radius: 0.3

local_costmap:
  robot_radius: 0.3
  inflation_radius: 0.55

global_costmap:
  robot_radius: 0.3
  inflation_radius: 0.55

controller_server:
  FollowPath:
    max_vel_x: 0.6        # G1 max walking speed
    min_vel_x: 0.0
    max_vel_theta: 0.8    # G1 max yaw rate
    max_speed_xy: 0.6
```

Add Nav2 to `hardware.launch.py`:
```python
nav2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
    ]),
    launch_arguments={
        'use_sim_time': 'false',
        'params_file': nav2_params_path,
        'map': '',  # empty = use SLAM live map
    }.items(),
)
```

**Test:** Set 2D Nav Goal in RViz → G1 should walk to it.

### Step 6: End-to-end navigation test (Day 8)
Run full stack, set 3 waypoints in the lab, verify G1:
- Plans a path in RViz
- Walks without hitting walls
- Recovers if it gets stuck

**Deliverable:** G1 navigates lab autonomously (no humans yet).

---

## Phase 3 — Human Avoidance (Days 9–14)

### Step 7: Validate human detection on hardware (Day 9)
```bash
ros2 launch g1_bringup hardware.launch.py
# In RViz: Add MarkerArray → /humans/markers
```
Stand in front of the camera. Verify:
- Human appears as red cylinder in RViz
- Position updates as you move
- Trajectory prediction (green line) appears when walking

Check detection latency:
```bash
ros2 topic hz /humans/markers  # should be ~10 Hz minimum
```
If slow, move YOLO inference to Jetson (use `yolo26n.engine` from Human_dtp).

### Step 8: Human costmap injection node (Days 10–11)
**New file:** `g1_detection/g1_detection/human_obstacle_node.py`

This node:
1. Subscribes to `/humans/markers` (detected human positions)
2. Publishes to Nav2's local costmap as dynamic obstacles
3. Uses `nav2_msgs/CostmapFilterInfo` or publishes to `/people` topic

```python
from nav2_msgs.msg import CostmapUpdate
from people_msgs.msg import People, Person

class HumanObstacleNode(Node):
    def __init__(self):
        super().__init__('human_obstacle_node')
        self._sub = self.create_subscription(
            MarkerArray, '/humans/markers', self._markers_cb, 10
        )
        self._pub = self.create_publisher(People, '/people', 10)

    def _markers_cb(self, msg):
        people = People()
        people.header.frame_id = 'map'
        people.header.stamp = self.get_clock().now().to_msg()
        for marker in msg.markers:
            if marker.ns == 'humans':
                p = Person()
                p.name = str(marker.id)
                p.position = marker.pose.position
                people.people.append(p)
        self._pub.publish(people)
```

Nav2 uses the `social_layer` or `people_obstacle_layer` plugin to inflate humans in the costmap.

Add to `nav2_params.yaml`:
```yaml
local_costmap:
  plugins: ['obstacle_layer', 'people_layer', 'inflation_layer']
  people_layer:
    plugin: 'nav2_costmap_2d::PeopleObstacleLayer'
    people_topic: /people
    person_radius: 0.5
```

**Test:** Stand on the planned path. G1 should reroute around you.

### Step 9: Velocity safety override (Day 12)
Already partially implemented in `g1_robot/robot_node.py`. Complete it:

```python
def _human_cmd_cb(self, msg):
    cmd = msg.data  # STOP / SLOW_DOWN / NORMAL_OPERATION
    if cmd == 'STOP':
        self._human_override = True
        self._robot.stop()
        self.get_logger().warn('Human too close — STOP')
    elif cmd == 'SLOW_DOWN':
        self._max_vel_scale = 0.4  # reduce to 40% speed
    elif cmd == 'NORMAL_OPERATION':
        self._human_override = False
        self._max_vel_scale = 1.0
```

Thresholds (from `g1_slam/config/rtabmap.yaml` / detection node params):
- `front_stop_distance_m: 1.8` — stop if human within 1.8m ahead
- `front_slow_distance_m: 3.8` — slow if human within 3.8m ahead
- `side_stop_distance_m: 0.8` — stop if human within 0.8m to side
- `immediate_stop_distance_m: 0.8` — emergency stop regardless of direction

**Test:** Walk directly toward G1. Verify it stops at ~1.8m. Step aside, verify it resumes.

### Step 10: Tune avoidance thresholds (Days 13–14)
Run repeated tests:
1. Human crosses path at 90° — does G1 reroute or stop?
2. Human walks alongside G1 — does it maintain safe distance?
3. Two humans blocking a corridor — does G1 wait or find another route?
4. Human removes themselves from path — does G1 resume within 2 seconds?

Adjust `detection_node` parameters:
```python
HumanXZPredictor(
    front_stop_distance_m=1.8,    # tune per lab
    front_slow_distance_m=3.8,
    side_stop_distance_m=0.8,
    agv_hold_max_frames=15,       # how long to hold stop after human clears
)
```

**Deliverable:** G1 reliably stops/reroutes in all 4 test scenarios.

---

## Phase 4 — Integration + Demo (Days 15–21)

### Step 11: Full system integration (Days 15–17)
Run complete stack with single command:
```bash
ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161
```
This should launch: SLAM + RealSense + Detection + Nav2 + G1 locomotion.

Fix any TF/timing issues:
- Check `ros2 run tf2_tools view_frames` for broken TF chains
- Ensure all nodes use `use_sim_time: false`
- Check topic latencies: `ros2 topic delay /humans/markers`

### Step 12: Demo scenario setup (Days 18–19)
Define fixed waypoints in the lab (save as a yaml or click in RViz):
- Waypoint A: starting position
- Waypoint B: across the room
- Waypoint C: back to start (loop)

Place 1-2 humans along the route. Record demo:
```bash
ros2 bag record -o demo_run \
  /map /tf /tf_static /humans/markers /cmd_vel /odom/raw \
  /camera/color/image_raw /rtabmap/cloud_map
```

### Step 13: Demo polish (Days 20–21)
RViz config to show:
- Map (occupancy grid)
- Robot model
- Human detection markers (red cylinders)
- Human trajectory predictions (green lines)
- Nav2 planned path (blue line)
- Camera feed

Save RViz config to `g1_bringup/config/demo.rviz`.

Single launch command for demo:
```bash
ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161 rviz:=true
```

---

## Launch Commands Reference

```bash
# Source workspace (add to ~/.bashrc)
source /opt/ros/humble/setup.bash
source ~/Desktop/g1_slam/ros2_ws/install/setup.bash

# Full hardware stack
ros2 launch g1_bringup hardware.launch.py robot_ip:=192.168.123.161

# SLAM only (no robot control)
ros2 launch g1_slam slam.launch.py launch_camera:=true

# Build workspace
cd ~/Desktop/g1_slam/ros2_ws
colcon build
source install/setup.bash

# Check all topics
ros2 topic list

# View TF tree
ros2 run tf2_tools view_frames && evince frames.pdf
```

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| G1 SDK version mismatch with firmware | Medium | High | Check firmware on day 1, pin SDK version |
| RealSense mount position wrong | Medium | High | Measure physical offset, update d435_joint in URDF |
| YOLO too slow on laptop | High | Medium | Use Jetson from Human_dtp for inference |
| Nav2 DWB controller too slow for G1 gait | Medium | Medium | Switch to RPP (Regulated Pure Pursuit) controller |
| Human detection misses fast-moving people | Medium | Medium | Lower confidence threshold, increase prediction steps |
| G1 battery life limits test time | High | Low | Keep charger on hand, save rosbags for offline tuning |
