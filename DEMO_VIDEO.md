# Demo Video — Annotated Egocentric Pipeline

Produces a single broadcast-ready video from the G1's own camera while it walks,
showing everything the stack knows:

| Layer | Source | Appearance |
|---|---|---|
| Human bounding boxes | `HumanXZPredictor` (Human_dtp), via `/g1/detections` | corner brackets + `HUMAN <id>  <dist>m` tag, colour-coded by threat state |
| Human trajectory forecast | Human_dtp's physics model, in the world frame | magenta dashed line on the floor + a ring at the 1.5 s horizon |
| Robot planned trajectory | Nav2 `/plan` (and `/local_plan`) | cyan ribbon projected onto the floor ahead |
| Robot true trajectory | accumulated `odom -> pelvis` TF | amber; **mainly visible in the BEV inset** (see below) |
| State | `/g1/human_cmd` | `CLEAR` / `SLOWING` / `STOP` chip; full red frame on STOP |
| Bird's-eye inset | all of the above | robot-centric, heading-up mini-map |

> **Why the true trajectory lives in the inset.** The path the robot has already
> walked is *behind* the camera, so in a forward-facing egocentric view it
> projects off-frame almost all the time. It is still drawn in the main view (it
> appears when the robot turns back on itself), but the bird's-eye inset is where
> it actually reads. That inset also makes the whole scene legible to an audience
> who can't mentally invert a perspective projection in real time.

---

## Run it

The overlay is on by default in both launchers.

```bash
# ZED end-to-end (the hardware path)
./run_zed_e2e.sh                                  # perception only
USE_ZED_ODOM=1 ./run_zed_e2e.sh                   # ZED VIO odometry
WITH_ROBOT=1 NET_IFACE=<nic> ./run_zed_e2e.sh     # + drive the G1

# watch it live
ros2 run rqt_image_view rqt_image_view /g1/overlay/image

# stop — ALWAYS via the script, so the MP4 is flushed and closed
./run_zed_e2e.sh stop
```

The video lands in `~/g1_demos/g1_demo_<timestamp>.mp4`. Override with
`DEMO_DIR=/path ./run_zed_e2e.sh`; disable with `OVERLAY=0`, or keep the live
view without writing a file with `RECORD=0`.

Via the launch file instead:

```bash
ros2 launch g1_bringup hardware.launch.py camera:=zed \
    overlay:=true record_video:=true video_path:=~/demos/take3.mp4
```

**Stop cleanly.** The MP4 is finalised when `overlay_node` shuts down. Killing
the terminal with `SIGKILL` leaves an unplayable file. `./run_zed_e2e.sh stop`
sends `SIGINT` first for this reason.

---

## Pre-flight — run this before the take that matters

```bash
./run_zed_e2e.sh                 # wait ~25 s for everything to settle
./tools/preflight_overlay.sh     # tells you which features you'll actually get
```

It checks, in the order things fail: camera + intrinsics, the
`odom -> <optical frame>` TF the AR layers depend on, whether detections are
flowing and carrying world data, whether any track has a forecast, `/plan`, and
the output topic rate. Nothing here has been run against real hardware yet, so
treat the first run as a shakedown, not the take.

### The two failure modes worth knowing

**Boxes render, but no magenta forecast lines.** Two causes:

1. *The person is standing still.* Human_dtp only forecasts a person who is
   **moving** — it discards heading samples below 0.08 m of travel and returns
   nothing if fewer than two survive. Verified: at 1.0 m/s and 0.2 m/s it emits
   20 forecast points; standing still it emits 0, even with 3 cm of depth noise.
   This is the Human_dtp model's own behaviour, not an overlay bug. **Have the
   person walk.**
2. *The `odom -> <optical frame>` TF doesn't resolve.* Then `world_frame` comes
   back `null` in the payload and every world-space layer is skipped. The node
   logs `No TF 'odom' <- '...'` once on startup. Boxes are unaffected.

**The MP4 won't open, or plays fast.** Both fixed, but the cause matters: the
file is only finalised when `overlay_node` runs its shutdown handler, so always
stop via `./run_zed_e2e.sh stop` (it now SIGINTs the overlay and waits for it
before tearing down the tmux session). Playback is paced to wall-clock, so a
slow render loop no longer produces a fast-motion video.

## Getting a good take

1. **Light the room well.** YOLO confidence drives whether boxes are solid or
   flagged `?` (unconfirmed track).
2. **Give it 10–15 s of walking before the human enters.** The travelled path
   and the BEV trail need history to look like anything, and Human_dtp needs
   ~3 frames of track history before it will forecast at all.
3. **Have the person walk *across* the robot's path, not straight at it.** A
   lateral crossing produces a forecast line that visibly diverges from the
   person's current position — that's the shot that shows prediction working. A
   head-on approach produces a forecast that just points at the camera.
4. **Send the goal well beyond the human** so `/plan` stays long and the cyan
   ribbon runs to the horizon.
5. Record several takes. It is one command per take and they are cheap.

---

## Tuning the look

All are `ros2 run g1_detection overlay_node --ros-args -p <name>:=<value>`:

| Parameter | Default | Notes |
|---|---|---|
| `render_rate_hz` | `15.0` | Output video frame rate. |
| `render_width` | `0` | `0` = native. Set `960` on the Jetson for headroom. |
| `bev_range_m` | `6.0` | Radius the inset covers. |
| `show_bev` | `true` | Turn the inset off for a clean egocentric-only shot. |
| `title` | `UNITREE G1 \| ...` | Banner text. **ASCII only** — OpenCV's font renders anything else as `?`. |
| `optical_frame` | *(auto)* | Camera optical frame; read from `camera_info` by default. Override if that frame is missing from the TF tree — the symptom is boxes rendering but no AR layers. Also available on `detection_node`. |
| `pelvis_height_m` | `0.75` | Floor height = pelvis Z minus this. See below. |
| `ground_z_mode` | `auto` | `fixed` + `ground_z` to pin the floor plane manually. |
| `detection_stale_s` | `0.7` | Older detections are dropped rather than drawn on the wrong frame. |

### If the AR paths float at eye level or sink through the floor

The floor is **not** `z = 0` in this stack. With ZED VIO the `odom` origin sits
at the camera's pose when tracking started — roughly 1.2 m up. The overlay
derives the floor as `pelvis_z_in_odom - pelvis_height_m`, so if the paths sit
too high or too low, adjust `pelvis_height_m` (the G1's pelvis is ~0.75 m off
the ground when standing), or pin it outright:

```bash
-p ground_z_mode:=fixed -p ground_z:=-1.15
```

### Performance

Rendering costs ~16 ms/frame at 1280×720 on a desktop CPU, ~22 ms with a long
travelled path. It runs on its own timer and reads `/g1/detections` rather than
running YOLO again, so it does not compete with the detector for GPU. On the
Jetson, set `render_width:=960` and/or `render_rate_hz:=10` if the pipeline FPS
readout in the HUD starts dropping.

---

## Iterating off-robot

The renderer (`g1_detection/overlay_render.py`) imports no ROS, so the look can
be developed without hardware:

```bash
# synthetic scene -> MP4 + stills; exercises every drawing path
python3 tools/preview_overlay.py --out /tmp/preview.mp4 --stills /tmp/stills

# hand-derived pinhole checks: axes, handedness, near-plane clipping
python3 tools/test_overlay_geometry.py

# world-frame track correctness + a regression guard on the camera-frame path
python3 tools/test_detection_world_track.py

# the /g1/detections contract between the two nodes, incl. degraded payloads
python3 tools/test_detection_payload.py
```

Run all four after touching `overlay_render.py`, `detection.py` or
`detection_node.py`. 57 checks total; they need only numpy and OpenCV.

---

## How it is wired

```
ZED  ──►  detection_node  ──►  /g1/detections   (JSON: bboxes, ids, world
 │         (Human_dtp)           │               positions, forecasts, cmd,
 │                               │               source frame stamp)
 │                               ▼
 └──────────────────────────►  overlay_node  ──►  /g1/overlay/image  +  MP4
                                 ▲   ▲
              Nav2 /plan ────────┘   └──────── TF odom->pelvis (true path)
                                               TF odom->camera optical
```

`overlay_node` buffers the last ~45 camera frames and draws each detection onto
the frame whose timestamp it was computed from, so boxes sit on the pixels YOLO
actually saw instead of lagging by the inference latency.

### The world-frame detail

`detection.py` carries **two** parallel tracks per person:

- `tracks_xz` — the long-standing camera-frame pair the avoidance thresholds in
  `_classify_agv_region` are tuned against. Untouched; still the only thing that
  feeds `/g1/human_cmd`.
- `tracks_world` — the same point pushed through a full 4×4 transform of the
  camera **optical** frame into the `odom` ground plane, and run through the same
  `_predict_future_xz` model. Visualisation only.

The second one exists because `_camera_xz_to_odom_xz` drops the optical Y axis
and then reads world rows 0 and 2 — i.e. it treats world *height* as a ground
axis, and applies a body-frame transform to optical-frame points. That is
harmless while the camera is level and TF is near identity (which is why
simulation passed), but on a walking robot it skews human world positions.
`human_obstacle_node.py` already documents this, which is why the costmap
injection is still commented out in `nav2_params_hw.yaml`.

Fixing it properly means re-tuning the front/side region logic that drives the
safety STOP, so it was deliberately left alone here — the demo overlay gets
correct geometry without touching the avoidance path. **If you later want
`/human_obstacle_cloud` enabled so Nav2 actually reroutes around forecast human
positions, that transform is the thing to fix first**, and `tracks_world` is the
correct version to switch it over to.
