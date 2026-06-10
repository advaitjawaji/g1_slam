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
| CUDA 13 active | `readlink -f /usr/local/cuda` → `/usr/local/cuda-13.0` |
| ZED Diagnostic clean | `/usr/local/zed/tools/ZED_Diagnostic -c` → `ZED SDK Diagnostic : OK`, `CUDA loading: true` |
| TensorRT builder lib reachable | `ls /usr/local/cuda-13.0/targets/x86_64-linux/lib/libnvinfer_builder_resource.so.10.13.2` |

If the SDK diagnostic or CUDA checks fail, see §6 — the toolkit/TRT setup must
be right or `Camera.open()` aborts with `CORRUPTED SDK INSTALLATION`.
If the ROS packages need rebuilding, see §6 — under CUDA 13 the patched cmake
flag is no longer required (kept only as a fallback for CUDA 12.5-only systems).

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
# Use zed2i for the ZED 2i unit on this machine (serial 34721371).
# Using zed2 still works but prints a model-mismatch warning.
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Wait for the log to get past `=== CAMERA OPENING ===` with no errors.
A repeating `CAMERA STREAM FAILED TO START` means the camera isn't visible →
back to §2.

First-run only: ZED will spend a few minutes optimizing the neural depth model
for your GPU (logs: `Optimizing model: neural_depth_light_5 ...`). This is
normal. The optimized engine is cached under `/usr/local/zed/resources/` and
subsequent launches start in seconds. If you see
`Unable to load library: libnvinfer_builder_resource.so.10.13.2`, fix the
loader path per §6.

### Terminal 2 — verify topics, then run detection

```bash
ros2 topic list | grep zed
```

Expect (verified against wrapper v5.3.1 on this machine — namespace is `/zed/zed_node/`):

```
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/rgb/color/rect/camera_info
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/depth_registered/camera_info
```

> Topic-name change: the wrapper used to publish `rgb/image_rect_color` and
> `rgb/camera_info`. Current v5.3.1 publishes `rgb/color/rect/image` and
> `rgb/color/rect/camera_info`. Always trust `ros2 topic list` over old docs.

Confirm data is actually flowing (each should report a steady rate, ~15–30 Hz):

```bash
ros2 topic hz /zed/zed_node/rgb/color/rect/image --window 20
ros2 topic hz /zed/zed_node/depth/depth_registered --window 20
```

Then launch the detection node. **Run it from the repo root** — `model_path` is
relative to the working directory and the cached weights live there:

```bash
ros2 run g1_detection detection_node --ros-args \
  -p model_path:=yolo26n.pt \
  -p color_topic:=/zed/zed_node/rgb/color/rect/image \
  -p depth_topic:=/zed/zed_node/depth/depth_registered \
  -p camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info \
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
| `Got intrinsics` never logged | Wrong `camera_info_topic`. Current name is `/zed/zed_node/rgb/color/rect/camera_info` (not the old `rgb/camera_info`). |
| `Camera model does not match user parameter ... 'zed2i'` | Launched with `camera_model:=zed2` but the device is a ZED 2i. Use `camera_model:=zed2i`. Non-fatal warning. |
| `Gravity alignment issues detected. Recomputing alignment...` | Informational — IMU is re-aligning. Hold the camera still ~2 s at startup to minimize. |
| `CRITICAL IMPORT ERROR. Run this from the ROOT folder.` (from detection_node) | Optional EigenTrajectory import failing (disabled by default per README). Safe to ignore; the node still runs. |
| `CORRUPTED SDK INSTALLATION` / `NEURAL CORRUPTED MODEL` | Stale TensorRT engines after a CUDA toolkit switch. Re-optimize: `sudo /usr/local/zed/tools/ZED_Diagnostic -aio && sudo /usr/local/zed/tools/ZED_Diagnostic -nrlo_all`. If that fails with `libnvinfer_builder_resource.so.10.13.2: cannot open`, fix loader path per §6. |
| `ZED Diagnostic`: `CUDA 13 not detected` but CUDA Operations OK | `/usr/local/cuda` symlink points at the wrong toolkit, or a second CUDA install confuses SDK 5.2.3. See §6. |
| `Multiple version of CUDA has been detected` warning + `CUDA loading: false` | ZED SDK 5.2.3 refuses to mark CUDA as loaded when a non-13 toolkit is present alongside. Hide or uninstall the other CUDA (§6). |
| Node runs, humans visible, but `/g1/human_cmd` stays `NORMAL_OPERATION` | Check depth: `ros2 topic hz` on the depth topic; person may be > 6 m or depth invalid (ZED min range ~0.3 m, struggles on textureless/backlit scenes). |
| `FileNotFoundError: yolo26n.pt` | You're not in the repo root (model path is cwd-relative). `cd ~/Desktop/g1_slam` or pass an absolute `model_path`. |
| Detection very slow / laggy | Confirm YOLO is on GPU: watch `nvidia-smi` while running. CPU-only ultralytics still works but at low FPS. |
| Rebuild fails: `Could NOT find CUDA: ... required is at least "13"` | Only on systems without CUDA 13. The ZED SDK cmake gotcha — see §6. |

## 6. CUDA / TensorRT / rebuilding (only if needed)

### 6.1 CUDA 13 toolkit setup (current state of this machine)

ZED SDK 5.2.3 requires the CUDA **13** toolkit on disk; the runtime only needs
the NVIDIA driver (CUDA 13.2 reported by `nvidia-smi`). Current install:

- `/usr/local/cuda-13.0` — active toolkit (nvcc 13.0.88)
- `/usr/local/cuda-12.5` — kept side-by-side for legacy builds (does not break ZED)
- `/usr/local/cuda` → symlink managed by `update-alternatives` → currently CUDA 13

If the diagnostic says `CUDA 13 not detected`, switch the alternative:

```bash
# Register CUDA 13 if it isn't already a choice
sudo update-alternatives --install /usr/local/cuda cuda /usr/local/cuda-13.0 130
sudo update-alternatives --config cuda     # pick /usr/local/cuda-13.0

# Make sure the loader sees CUDA 13 libs
sudo tee /etc/ld.so.conf.d/cuda-13.conf >/dev/null <<'EOF'
/usr/local/cuda-13.0/targets/x86_64-linux/lib
/usr/local/cuda-13.0/lib64
EOF
sudo ldconfig

readlink -f /usr/local/cuda                 # should print /usr/local/cuda-13.0
/usr/local/zed/tools/ZED_Diagnostic -c
```

If the runtime libs aren't installed yet:

```bash
sudo apt install -y cuda-toolkit-13-0 \
  libcublas-13-0 libnpp-13-0 cuda-nvrtc-13-0 \
  libcufft-13-0 libcurand-13-0 libcusolver-13-0 libcusparse-13-0
sudo ldconfig
```

### 6.2 TensorRT builder loader path (required for neural depth / OD optimization)

The TensorRT builder library `libnvinfer_builder_resource.so.10.13.2` ships
with no SONAME — `ldconfig` deliberately does **not** index it, so `dlopen()`
only finds it via `LD_LIBRARY_PATH` or RUNPATH. Symptom when missing:

```
createInferBuilder: Error Code 6: API Usage Error
  (Unable to load library: libnvinfer_builder_resource.so.10.13.2:
   cannot open shared object file)
Model optimization failed
CORRUPTED SDK INSTALLATION
```

Fix system-wide (recommended — works for ROS launches from any shell):

```bash
sudo tee /etc/profile.d/cuda13-trt.sh >/dev/null <<'EOF'
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/targets/x86_64-linux/lib:$LD_LIBRARY_PATH
EOF
# Open a new shell so the export takes effect.
```

Or per-shell:

```bash
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/targets/x86_64-linux/lib:$LD_LIBRARY_PATH
```

Or symlink into a default search dir (last resort):

```bash
sudo ln -sf /usr/local/cuda-13.0/targets/x86_64-linux/lib/libnvinfer_builder_resource.so.10.13.2 \
            /usr/lib/x86_64-linux-gnu/libnvinfer_builder_resource.so.10.13.2
```

After the path is fixed, re-launch the wrapper once and let it optimize
neural depth / object-detection models (a few minutes, one time). To force a
rebuild of all engines:

```bash
sudo /usr/local/zed/tools/ZED_Diagnostic -aio       # AI models
sudo /usr/local/zed/tools/ZED_Diagnostic -nrlo_all  # neural depth modes
```

### 6.3 Rebuilding ROS packages

Under CUDA 13 the ZED wrapper builds with the stock cmake config — no flag
needed:

```bash
cd ~/Desktop/g1_slam/ros2_ws
colcon build --symlink-install --packages-up-to zed_wrapper
colcon build --symlink-install --packages-select g1_detection
```

Fallback for CUDA 12.5-only systems (the original gotcha — SDK cmake demands
CUDA 13 strings but the SDK binary itself only needs the driver):

```bash
cd ~/Desktop/g1_slam/ros2_ws
colcon build --symlink-install --packages-up-to zed_wrapper \
  --cmake-args -DZED_DIR=$PWD/cmake/zed
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
