#!/usr/bin/env python3
"""
Regression + correctness checks for the world-frame track added to
`HumanXZPredictor` for the demo overlay.

Two things must hold:
  1. the long-standing camera-frame path that /g1/human_cmd is tuned against is
     bit-identical to what it produced before the refactor, and
  2. the new world-frame track is geometrically correct.

Runs without ROS, YOLO or a camera — `python3 tools/test_detection_world_track.py`.
`ultralytics` and `torch` are stubbed so the module imports on any machine.
"""
import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "g1_detection"))

# ── Stub the heavy optional deps before importing detection.py ────────────
_ul = types.ModuleType("ultralytics")
_ul.YOLO = lambda *a, **k: object()
sys.modules.setdefault("ultralytics", _ul)

from g1_detection.detection import HumanXZPredictor  # noqa: E402

checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))


def make_predictor(**kw):
    p = HumanXZPredictor(model_path="stub", draw=False, profile=False,
                         min_history_for_prediction=3, predict_every_n=1,
                         min_confirm_frames=2, **kw)
    p.model = None
    return p


# ── 1. Patch extraction: refactored xyz vs. the original xz implementation ──
def original_extract_xz(self, point_cloud_xyz, cx, cy, box=None):
    """Verbatim copy of the pre-refactor method, as the reference."""
    if point_cloud_xyz is None:
        return None
    h, w = point_cloud_xyz.shape[:2]
    if box is not None:
        x1, y1, x2, y2 = map(int, box)
        bw, bh = max(1, x2 - x1), max(1, y2 - y1)
        cx = int(x1 + 0.50 * bw)
        cy = int(y1 + 0.65 * bh)
        half_w = max(1, int(self.half_patch_scale_w * bw))
        half_h = max(1, int(self.half_patch_scale_h * bh))
    else:
        half_w = half_h = 2
    cx = max(0, min(cx, w - 1))
    cy = max(0, min(cy, h - 1))
    px1, px2 = max(0, cx - half_w), min(w, cx + half_w + 1)
    py1, py2 = max(0, cy - half_h), min(h, cy + half_h + 1)
    patch = point_cloud_xyz[py1:py2, px1:px2, :3]
    if patch.size == 0:
        return None
    patch = patch.reshape(-1, 3)
    x_vals, z_vals = patch[:, 0], patch[:, 2]
    valid = np.isfinite(x_vals) & np.isfinite(z_vals) & (z_vals > 0.0)
    if self.max_depth_m is not None:
        valid &= (z_vals <= self.max_depth_m)
    if not np.any(valid):
        return None
    return float(np.median(x_vals[valid])), float(np.median(z_vals[valid]))


pred = make_predictor()
rng = np.random.default_rng(7)
mismatches = 0
none_agree = 0
for trial in range(300):
    cloud = rng.normal(0.0, 2.0, size=(64, 96, 3)).astype(np.float32)
    # Sprinkle in the degenerate values the real depth stream produces.
    cloud[rng.random(cloud.shape[:2]) < 0.15] = np.nan
    cloud[rng.random(cloud.shape[:2]) < 0.10, 2] = -1.0
    cloud[rng.random(cloud.shape[:2]) < 0.05, 2] = 99.0
    box = [rng.integers(0, 60), rng.integers(0, 40),
           rng.integers(61, 96), rng.integers(41, 64)]
    got = pred._extract_xz_from_pointcloud(cloud, 0, 0, box=box)
    want = original_extract_xz(pred, cloud, 0, 0, box=box)
    if got is None or want is None:
        none_agree += (got is None) == (want is None)
        mismatches += (got is None) != (want is None)
        continue
    if not (np.isclose(got[0], want[0], equal_nan=True)
            and np.isclose(got[1], want[1], equal_nan=True)):
        mismatches += 1

check("xz extraction unchanged after xyz refactor (300 random clouds)",
      mismatches == 0, f"{mismatches} mismatches")

# ── 2. The world transform ────────────────────────────────────────────────
# Camera 1.2 m up looking along world +x; optical (x right, y down, z fwd).
R_BASE = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_wo = np.eye(4)
T_wo[:3, :3] = R_BASE
T_wo[:3, 3] = [0.0, 0.0, 1.2]

# A point 3 m ahead, 0.5 m to the camera's right, 0.4 m below the lens.
got = HumanXZPredictor._camera_xyz_to_world_xy(0.5, 0.4, 3.0, T_wo)
check("world transform: forward maps to +x", np.isclose(got[0], 3.0), f"x={got[0]}")
check("world transform: optical +x (right) maps to -y", np.isclose(got[1], -0.5), f"y={got[1]}")
check("world transform: optical +y (down) lowers z",
      np.isclose(got[2], 1.2 - 0.4), f"z={got[2]}")
check("world transform: rejects a missing transform",
      HumanXZPredictor._camera_xyz_to_world_xy(1, 1, 1, None) is None)
check("world transform: rejects a non-finite transform",
      HumanXZPredictor._camera_xyz_to_world_xy(1, 1, 1, np.full((4, 4), np.nan)) is None)

# ── 3. End-to-end: world track + prediction through process_frame ─────────
# A person walking in a straight line across the camera's view. Feed the same
# motion in and confirm the world prediction extrapolates along it.
pred = make_predictor()
H, W = 64, 96
BOX = np.array([30.0, 10.0, 60.0, 55.0], dtype=np.float32)
pred._parse_yolo_results = lambda r: [{"track_id": 1, "box": BOX}]
pred._run_model = lambda f: ["stub"]

frame = np.zeros((H, W, 3), np.uint8)
truth = []
for i in range(14):
    t = i * 0.1
    # Camera-optical: person 3 m ahead, sliding right at 1 m/s.
    x_cam, y_cam, z_cam = -0.5 + 1.0 * t, 0.3, 3.0
    cloud = np.zeros((H, W, 3), np.float32)
    cloud[:, :, 0], cloud[:, :, 1], cloud[:, :, 2] = x_cam, y_cam, z_cam
    _, out = pred.process_frame(frame, cloud, timestamp=t,
                                T_odom_camera=None, T_world_optical=T_wo)
    truth.append((x_cam, z_cam))

det = out["detections"][0]
check("detection carries a camera-frame position", det.get("position_camera_m") is not None)
check("detection carries a world position", det.get("position_world_m") is not None)

pw = det["position_world_m"]
# Person is 3 m ahead => world x ~ 3; sliding to the camera's right => world y
# decreasing. EMA smoothing lags the truth, so compare direction, not equality.
check("world x tracks forward distance", 2.5 < pw["x"] < 3.5, f"x={pw['x']:.3f}")
check("world z sits below the camera", pw["z"] < 1.2, f"z={pw['z']:.3f}")

hist = np.array(pred.tracks_world[1])
check("world history accumulates", len(hist) >= 10, f"n={len(hist)}")
check("world y decreases as the person moves right",
      hist[-1, 1] < hist[0, 1], f"{hist[0,1]:.3f} -> {hist[-1,1]:.3f}")
check("world x stays ~constant (person keeps their distance)",
      abs(hist[-1, 0] - hist[0, 0]) < 0.35, f"{hist[0,0]:.3f} -> {hist[-1,0]:.3f}")

pworld = det.get("predictions_world", [])
check("world prediction is produced", len(pworld) > 0, f"n={len(pworld)}")
if pworld:
    check("world prediction continues the direction of travel",
          pworld[-1]["y"] < pw["y"], f"now={pw['y']:.3f} pred={pworld[-1]['y']:.3f}")
    horizon = pworld[-1]["t"] - pworld[0]["t"]
    check("prediction horizon is about 1.5 s", 1.0 < horizon <= 1.6, f"{horizon:.2f}s")

# ── 4. The legacy path still works when there is no world transform ───────
pred2 = make_predictor()
pred2._parse_yolo_results = lambda r: [{"track_id": 1, "box": BOX}]
pred2._run_model = lambda f: ["stub"]
for i in range(8):
    cloud = np.zeros((H, W, 3), np.float32)
    cloud[:, :, 0], cloud[:, :, 1], cloud[:, :, 2] = 0.2, 0.3, 2.0
    _, out2 = pred2.process_frame(frame, cloud, timestamp=i * 0.1)

d2 = out2["detections"][0]
check("legacy xz path unaffected with no world TF",
      abs(d2["position_xz_m"]["z"] - 2.0) < 0.5, f"z={d2['position_xz_m']['z']:.3f}")
check("world fields degrade to empty without a world TF",
      d2.get("position_world_m") is None and d2.get("predictions_world") == [])
check("avoidance command still computed",
      out2["agv_movement"]["command"] in ("stop", "slow_down", "normal_operation"),
      out2["agv_movement"]["command"])

# ── 5. Track cleanup clears the new state too ─────────────────────────────
pred._reset_track(1)
check("reset_track clears world state",
      1 not in pred.tracks_world and 1 not in pred.last_predictions_world
      and 1 not in pred._world_smooth)

# ── Report ────────────────────────────────────────────────────────────────
width = max(len(n) for n, _, _ in checks)
failed = 0
for name, ok, detail in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {name.ljust(width)}  {detail}")
    failed += (not ok)
print(f"\n{len(checks) - failed}/{len(checks)} passed")
sys.exit(1 if failed else 0)
