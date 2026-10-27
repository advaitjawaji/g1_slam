# Trial Recording

Every launch of `run_zed_e2e.sh` records a complete trial into its own folder:
the rosbag, every component's console output, ROS node logs, live parameters,
the TF tree, the G1's onboard state, the overlay video, and the exact code and
software versions that ran. Turn it off with `RECORD_TRIAL=0`.

> **Status: not yet run on hardware.** Treat the first trial as a shakedown and
> read the `trial_check` report it prints on stop.

---

## Running a trial

```bash
# 1. Rebuild after pulling: the installed build is what runs, not src/.
cd ~/Desktop/g1_slam/ros2_ws && colcon build --packages-select g1_detection g1_robot && cd ..

# 2. Launch with trial metadata (all TRIAL_* are optional; edit trial.yaml later)
TRIAL_NAME=corridor_2ped TRIAL_ENV=corridor TRIAL_PEDS=2 TRIAL_PED_SPEED=normal \
  WITH_ROBOT=1 NET_IFACE=<nic> USE_ZED_ODOM=1 ./run_zed_e2e.sh

# 3. Wait for "Snapshot ... done" in tmux window 'trial' (~65 s), then run:
python3 tools/send_goal.py 3.0 0.0                                   # logged goal
python3 tools/trial_event.py --kind ped_start "P1 starts walking"    # mark moments
python3 tools/trial_event.py --kind failure "robot did not stop for P1"

# 4. Stop. This flushes the bag, video and state CSVs, then checks the trial.
./run_zed_e2e.sh stop
python3 tools/trial_check.py ~/g1_trials/<folder>                    # re-check anytime
```

| Variable | Default | Effect |
|---|---|---|
| `RECORD_TRIAL` | `1` | `0` = no trial folder (previous behaviour) |
| `TRIAL_ROOT` | `~/g1_trials` | Where trial folders go |
| `TRIAL_NAME` | *(empty)* | Suffix on the folder name |
| `TRIAL_ENV`, `TRIAL_PEDS`, `TRIAL_PED_SPEED`, `TRIAL_NOTES` | *(empty)* | Pre-fill `trial.yaml` |
| `TRIAL_META` | *(empty)* | Copy a pre-filled `trial.yaml` instead |
| `BAG_IMAGES` | `compressed` | `raw` adds full RGB + depth (large); `none` records no images |
| `BAG_CLOUD` | `0` | `1` adds the ZED point cloud (very large) |
| `BAG_EXTRA_TOPICS` | *(empty)* | Extra topics, e.g. motion-capture poses for ground truth |
| `SVO` | `1` | ZED native SVO recording (full stereo + IMU, replayable in the ZED SDK); `0` = off |
| `SVO_COMPRESSION` | `2` | Wrapper v5.3.1 mapping: `1` H.264, `2` H.265, `3`/`4`/`5` lossless variants (very large) |

## What a trial folder holds

```
~/g1_trials/<YYYYmmdd_HHMMSS>_<TRIAL_NAME>/
├── trial.yaml            operator metadata: environment, pedestrians, speeds, outcome, notes
├── launch_info.yaml      launcher toggles, git commit, dirty flag, start/end time
├── events.jsonl          goals and operator marks (also on /g1/trial/events in the bag)
├── bag/                  rosbag2; topic list in bag_topics.txt
├── camera.svo2           ZED native recording: stereo video + IMU + timestamps (svo.log = service replies)
├── overlay.mp4           the annotated egocentric video
├── system.csv            computer load: CPU, memory, temperatures, GPU, per-component CPU/RAM
├── robot_state/          the G1's onboard state (see below)
├── logs/                 console output of every tmux pane
├── ros_logs/             ROS 2 node log files
├── snapshot/             nodes, topics, live params of every node, TF tree, odom->pelvis
└── code/                 git commit/status/diff, source tarball, install-vs-src check, versions
```

## What is recorded from each source

| Source | Recorded | Where |
|---|---|---|
| G1 joints (all 35 motor slots) | position, velocity, acceleration, estimated torque, every message | `robot_state/lowstate.csv` |
| G1 motor health | mode, two temperatures, voltage, sensor words, error flags (`motorstate`), 10 Hz **and** at once on any mode/flag change | `robot_state/motor_health.csv` |
| G1 IMUs | pelvis IMU (quaternion, gyro, accel, rpy) every message; IMU temperature; torso IMU (`rt/secondary_imu`) | `lowstate.csv`, `motor_health.csv`, `secondary_imu.csv` |
| G1 other | robot tick and modes, raw remote bytes (operator input); onboard pose/velocity if published; any other state topic via `--extra` | `robot_state/` |
| ZED camera | SVO (stereo + IMU, replayable to regenerate depth, tracking, point clouds); compressed left RGB; camera_info; temperatures; health/heartbeat | `camera.svo2`, `bag/` |
| ZED IMU / tracking | `imu/data`, `imu/data_raw`, magnetometer, barometer, odometry, pose with covariance, tracking status | `bag/` |
| Perception / safety | per-frame boxes, tracks, ranges, forecasts, command, timing; commands actually sent to the robot | `bag/` (`/g1/detections`, `/g1/robot_cmd`) |
| Navigation | goal, global plan, RPP look-ahead point and collision arc, both costmaps and footprints, Nav2 output velocities, behaviour-tree log, action status | `bag/` |
| Computer | CPU (mean and busiest core), load, memory, CPU temperature, GPU utilisation / memory / temperature / power / clock, CPU % and RAM per pipeline component, 2 Hz | `system.csv` |
| Provenance | git commit, diff, source tarball, install-vs-src check, versions, live parameters, TF tree, console and ROS logs | `code/`, `snapshot/`, `logs/`, `ros_logs/` |

## Where each metric's data comes from

All ROS timestamps and `robot_state` rows share the host clock
(`use_sim_time:=false`), so everything lines up on one timeline.

| Metric | Data | Caveat |
|---|---|---|
| Time to detect a person | First `/g1/detections` frame containing the track (`stamp` = image capture time), against when the person became visible in the recorded images | "Became visible" needs image annotation or external ground truth |
| Capture → STOP decision | `/g1/detections`: `stamp` → `t_done`. `t_rx` and `timings_ms` split it into queueing, inference and post-processing | `state` names the branch that fired (current position vs forecast) |
| Decision → stop command sent | `t_done` of the first STOP frame → `/g1/robot_cmd` `human_cmd.t_rx` → `stop_move.t_call` | |
| Stop command → robot at rest | `/g1/robot_cmd` `stop_move` → `robot_state/lowstate.csv` (joint `dq`, IMU) and `/odom` twist | ZED odometry smooths through the gait; mocap is better |
| Stopping distance | `/odom` pose (or mocap via `BAG_EXTRA_TOPICS`) from `stop_move` to rest; grouped by `trial.yaml` pedestrian count and speed | |
| Pedestrian speed | `/g1/detections` `world` positions (odom frame) over time | Inherits ZED odometry error; instructed speed is in `trial.yaml` |
| Tracking performance | Goal: `/g1/trial/goal` + `goal_sent`. Plan: `/plan`, `/local_plan`. Commanded: `/cmd_vel` (Nav2) and `/g1/robot_cmd` `sent` (after the override). Achieved: `/odom`. Outcome: `goal_result`, `/navigate_to_pose/_action/status`. Recoveries: `/behavior_tree_log` | |
| Human–robot separation and its cause | `/g1/detections` per person per frame: `dist`, `xz`, `world`, `pred_safety`, `state`, `cmd`, alongside the robot timeline above | `dist` is camera-to-person along the tilted optical axis, not body-to-body |
| Cross-environment comparison | `trial.yaml` `environment`; every trial has the same layout | |
| Required separation (ISO) | Measured inputs: reaction time (latency chain), stopping time and distance, robot speed (`/odom`), pedestrian speed, detection position error (from detection-accuracy trials) | Check the formula against the standard's own text before use |
| Failure cases and root cause | All of the above + `/rosout`, `logs/`, `ros_logs/`, `/diagnostics`, costmaps, live params, code state, `--kind failure` events | |

### New fields used above

- `/g1/detections` (JSON, per processed frame): `t_rx`, `t_done`, `depth_stamp_ns`,
  `map_tf_ok`, `frame_idx`, `inference_ran`, `timings_ms`, and per person `xz` and
  `pred_safety`: the position and forecast the STOP/SLOW decision actually used.
  (`world`/`pred` are a separate track that exists only for visualisation.)
- `/g1/robot_cmd` (JSON, per event, from `robot_node`): `human_cmd` (received
  command, override state, scale), `stop_move` (`t_call`), `cmd_vel` (velocity in,
  whether blocked, velocity actually sent after scaling and clamping). `t` minus
  `t_call` is how long the SDK call blocked. The `ret` field is always `null`:
  the SDK's `Move()` and `StopMove()` discard the robot's reply code.

## What recording cannot do

- **Ground truth.** Nothing here measures the robot or pedestrians independently.
  If the mocap system has a ROS 2 driver, add its topics with `BAG_EXTRA_TOPICS`.
  Otherwise sync an external camera with a visible event marked by
  `trial_event.py`.
- **Scripted stops.** `detection_node` republishes `/g1/human_cmd` every frame, so a
  STOP published by hand is overwritten within one frame. Stopping trials need a
  real trigger (a person) or a dedicated stop input, which has not been added.
- **Zero overhead.** The recorders share the machine with detection, and SVO
  encoding uses the GPU's video encoder alongside YOLO. `system.csv` shows the
  cost per component; compare the "processed X% of camera frames" line from
  `trial_check` against a run with `RECORD_TRIAL=0`. `SVO=0` and `BAG_IMAGES=none`
  lower the load.
- **Battery state.** The SDK defines a battery message (`BmsState_`), but no
  Unitree G1 example names its topic, so it is not subscribed by default. Once
  the topic name is confirmed on the robot, add it to the logger with
  `--extra <topic>=unitree_hg.BmsState_`.
- **YOLO confidence scores.** Not recorded live; `/g1/detections` `conf` means
  "track confirmed". They can be recovered by re-running the detector offline on
  `camera.svo2`.

## Changes to the validated pipeline

Recording required small, publish-only additions to code that ran on hardware.
The STOP/SLOW decision logic and the commands sent to the robot are unchanged:

- `detection_node.py`: extra JSON fields on `/g1/detections`; `profile=True` (adds
  `timings_ms` only).
- `robot_node.py`: new `/g1/robot_cmd` log publisher.
- `g1.py`: records the last command sent and the SDK return value.

`tools/test_detection_payload.py`, `tools/test_detection_world_track.py` and
`tools/test_state_logger_rows.py` pass with these changes.
