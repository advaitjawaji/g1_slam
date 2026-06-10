# ZED 2 Standalone Human-Detection Test — Lab Procedure

> Step-by-step guide for running the real YOLO detection pipeline against a ZED 2
> camera, **no robot, no SLAM, no Nav2**. This closes STATE.md §9 step 1 — the first
> run of `detection_node` with real perception in this repo's loop.
> Written 2026-06-10. Pairs with `STATE.md` §6 (camera support) and §8 (gotchas).

## 0. What you are testing

```
ZED 2 ──USB 3──▶ zed_wrapper node ──▶ /zed/zed_node/{rgb, depth, camera_info}
                                          │
                                          ▼
                                    detection_node  (YOLO26n + depth back-projection
                                          │          + Kalman tracking + prediction)
                                          ├──▶ /g1/human_cmd      ← the signal you care about
                                          └──▶ /humans/markers    ← RViz visualization
```

Success = `/g1/human_cmd` flips between `NORMAL_OPERATION` / `SLOW_DOWN` / `STOP`
as you walk toward and away from the camera.

## 1. Pre-lab checklist (all already done on this machine — verify, don't redo)

| Item | How to verify |
|---|---|
| ZED SDK 5.2.3 | `ls /usr/local/zed` exists |
| zed-ros2-wrapper v5.3.1 built | `source ros2_ws/install/setup.bash && ros2 pkg list \| grep zed` → 4 packages |
| g1_detection built | same source, `ros2 pkg executables g1_detection` |
| YOLO weights cached (no internet needed) | `ls ~/Desktop/g1_slam/yolo26n.pt` (5.3 MB) |
| ultralytics installed | `python3 -c "import ultralytics"` |

If any of these fail, see §6 (rebuild) — **do not** plain `colcon build`; the ZED
package needs a special cmake flag (CUDA gotcha, STATE.md §8).

## 2. Hardware setup

1. Plug the ZED 2 into a **USB 3 port** (blue tab). USB 2 will enumerate but
   streams fail or run crippled.
2. Sanity-check **outside ROS first** so camera problems can't be confused with
   ROS problems:
   ```bash
   /usr/local/zed/tools/ZED_Diagnostic     # all checks should pass
   /usr/local/zed/tools/ZED_Explorer      # you should see live video
   ```
   If `ZED_Explorer` shows no image, fix that before touching ROS (try another
   USB 3 port / cable; check `lsusb | grep -i stereolabs`).
3. First-ever camera use on a machine downloads a factory calibration file —
   this is the **one step that needs internet**. If the lab has no wifi, run
   `ZED_Explorer` once beforehand wherever there is internet.
4. Mount/hold the camera roughly level, ~1 m or higher off the ground, with a
   clear 4–5 m corridor in front of it to walk in.

## 3. Run the test — three terminals

All terminals start with:

```bash
cd ~/Desktop/g1_slam
source ros2_ws/install/setup.bash
```

### Terminal 1 — ZED wrapper

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

Wait for the log to get past `=== CAMERA OPENING ===` with no errors.
A repeating `CAMERA STREAM FAILED TO START` means the camera isn't visible →
back to §2.

### Terminal 2 — verify topics, then run detection

```bash
ros2 topic list | grep zed
```

Expect (verified against wrapper v5.3.1 on this machine — namespace is `/zed/zed_node/`):

```
/zed/zed_node/rgb/image_rect_color
/zed/zed_node/depth/depth_registered
/zed/zed_node/rgb/camera_info
```

Confirm data is actually flowing (each should report a steady rate, ~15–30 Hz):

```bash
ros2 topic hz /zed/zed_node/rgb/image_rect_color --window 20
ros2 topic hz /zed/zed_node/depth/depth_registered --window 20
```

Then launch the detection node. **Run it from the repo root** — `model_path` is
relative to the working directory and the cached weights live there:

```bash
ros2 run g1_detection detection_node --ros-args \
  -p model_path:=yolo26n.pt \
  -p color_topic:=/zed/zed_node/rgb/image_rect_color \
  -p depth_topic:=/zed/zed_node/depth/depth_registered \
  -p camera_info_topic:=/zed/zed_node/rgb/camera_info \
  -p camera_frame:=zed_left_camera_frame
```

Expected startup logs, in order:

```
Subscribed to color='...' depth='...' info='...'
Detection node ready (waiting for camera_info).
Got intrinsics: fx=... fy=... cx=... cy=...
```

If `Got intrinsics` never appears, the camera_info topic name is wrong —
re-check `ros2 topic list`.

> Standalone note: there is **no TF tree** in this test. That's fine —
> the node handles a missing `map → camera` transform by reporting positions in
> the camera frame, and `/g1/human_cmd` is computed from camera-frame X/Z
> regardless. The `camera_frame`/`map_frame` params only matter for the full
> robot stack.

### Terminal 3 — watch the output

```bash
ros2 topic echo /g1/human_cmd
```

## 4. Test script — what to do in front of the camera, what you should see

Distances are from the camera, measured along the optical (forward/Z) axis.
The "front zone" is a ±0.7 m wide corridor straight ahead; everything else is "side".

| You do | Expected `/g1/human_cmd` |
|---|---|
| Nobody in view | `NORMAL_OPERATION` |
| Stand directly ahead at ~5 m | `NORMAL_OPERATION` (front slow threshold is 3.8 m) |
| Walk in to ~3 m, front | `SLOW_DOWN` |
| Walk in to ~1.5 m, front | `STOP` (front stop threshold 1.8 m) |
| Anywhere closer than 0.8 m | `STOP` (immediate-stop override) |
| Stand 1.5 m away but well off to the side | `SLOW_DOWN` (side: stop < 0.8 m, slow ≤ 2.0 m) |
| Walk back out past 4 m | returns to `NORMAL_OPERATION` |

Notes on behavior:
- A track must be seen for **3 consecutive frames** before it's confirmed, so
  there's a fraction of a second of latency when you first enter the view.
- The pipeline also evaluates the **predicted** trajectory (1.5 s lookahead) —
  walking *briskly toward* the camera can trigger `SLOW_DOWN`/`STOP` slightly
  earlier than the static distances above. That is correct behavior, not a bug.
- Detections beyond 6 m are ignored (`max_depth_m`).

### Optional: visualize in RViz

```bash
rviz2
```

Set **Fixed Frame** to `map`, add a `MarkerArray` display on `/humans/markers`.
You should see a red cylinder per person and a green line strip for the
predicted path. (Without TF the coordinates are camera-frame, so placement
looks rotated — ignore that; geometry is only meaningful in the full stack.)

### Optional but recommended: record a bag for offline debugging

```bash
ros2 bag record -o zed_lab_test \
  /zed/zed_node/rgb/image_rect_color \
  /zed/zed_node/depth/depth_registered \
  /zed/zed_node/rgb/camera_info \
  /g1/human_cmd /humans/markers
```

A few minutes of walking is enough; this lets us replay and tune thresholds at
the desk instead of in the lab.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `CAMERA STREAM FAILED TO START` loop | Camera not visible to SDK. USB 2 port, bad cable, or another process holds the camera. Verify with `ZED_Explorer` (§2). |
| No `/zed/...` topics | Wrapper not sourced or crashed — re-check Terminal 1 output. |
| Topics exist but names differ from §3 | Wrapper version drift. Use whatever `ros2 topic list` shows; the detection params accept any names. |
| `Got intrinsics` never logged | Wrong `camera_info_topic`. |
| Node runs, humans visible, but `/g1/human_cmd` stays `NORMAL_OPERATION` | Check depth: `ros2 topic hz` on the depth topic; person may be > 6 m or depth invalid (ZED min range ~0.3 m, struggles on textureless/backlit scenes). |
| `FileNotFoundError: yolo26n.pt` | You're not in the repo root (model path is cwd-relative). `cd ~/Desktop/g1_slam` or pass an absolute `model_path`. |
| Detection very slow / laggy | Confirm YOLO is on GPU: watch `nvidia-smi` while running. CPU-only ultralytics still works but at low FPS. |
| Rebuild fails: `Could NOT find CUDA: ... required is at least "13"` | The ZED SDK cmake gotcha — see §6. |

## 6. Rebuilding (only if needed)

The ZED SDK 5.2.3 cmake config demands the CUDA **13** toolkit, but this machine
has the CUDA **12.5** toolkit (the SDK binary itself only needs the NVIDIA
driver, so building against 12.5 is safe). A patched config lives at
`ros2_ws/cmake/zed/`. Therefore:

```bash
cd ~/Desktop/g1_slam/ros2_ws
colcon build --symlink-install --packages-up-to zed_wrapper \
  --cmake-args -DZED_DIR=$PWD/cmake/zed
# g1 packages alone don't need the flag:
colcon build --symlink-install --packages-select g1_detection
```

On a **fresh checkout**, the wrapper source is gitignored — re-clone first:

```bash
git clone --recurse-submodules --depth 1 \
  https://github.com/stereolabs/zed-ros2-wrapper.git ros2_ws/src/zed-ros2-wrapper
sudo apt-get install -y ros-humble-zed-msgs ros-humble-zed-description \
  ros-humble-robot-localization ros-humble-geographic-msgs ros-humble-nmea-msgs \
  ros-humble-backward-ros ros-humble-image-transport-plugins \
  ros-humble-compressed-image-transport ros-humble-compressed-depth-image-transport \
  ros-humble-theora-image-transport
```

## 7. After the test — record results in STATE.md

Update STATE.md §2: move "`detection_node` / `HumanXZPredictor` (real YOLO)"
from *not validated* to *validated* (or record what failed), and note observed
FPS, detection range, and any threshold tuning needed. That keeps the
agent-readable status accurate for the next session.
