#!/usr/bin/env python3
"""
Contract test for the /g1/detections seam.

`detection_node` serialises the Human_dtp result to JSON; `overlay_node`
deserialises it and builds `overlay_render.Human` objects. The two live in
different processes, so nothing type-checks that seam — this does, using the
real predictor output rather than a hand-written fixture.

Runs without ROS — `python3 tools/test_detection_payload.py`.
"""
import json
import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "g1_detection"))

_ul = types.ModuleType("ultralytics")
_ul.YOLO = lambda *a, **k: object()
sys.modules.setdefault("ultralytics", _ul)

from g1_detection.detection import HumanXZPredictor      # noqa: E402
from g1_detection import overlay_render as R             # noqa: E402

checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))


# ── Produce a real predictor result ───────────────────────────────────────
R_BASE = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_wo = np.eye(4)
T_wo[:3, :3] = R_BASE
T_wo[:3, 3] = [0.0, 0.0, 1.2]

pred = HumanXZPredictor(model_path="stub", draw=False, profile=False,
                        min_history_for_prediction=3, predict_every_n=1,
                        min_confirm_frames=2)
pred.model = None
BOXES = [np.array([30.0, 10.0, 60.0, 55.0], dtype=np.float32),
         np.array([10.0, 12.0, 28.0, 50.0], dtype=np.float32)]
pred._parse_yolo_results = lambda r: [{"track_id": i + 1, "box": b}
                                      for i, b in enumerate(BOXES)]
pred._run_model = lambda f: ["stub"]

H, W = 64, 96
frame = np.zeros((H, W, 3), np.uint8)
for i in range(14):
    t = i * 0.1
    cloud = np.zeros((H, W, 3), np.float32)
    cloud[:, :, 0] = -0.5 + 1.0 * t
    cloud[:, :, 1] = 0.3
    cloud[:, :, 2] = 3.0
    _, result = pred.process_frame(frame, cloud, timestamp=t,
                                   T_odom_camera=None, T_world_optical=T_wo)

check("predictor produced detections", len(result["detections"]) > 0,
      f"n={len(result['detections'])}")


# ── Serialise exactly as detection_node._publish_detections does ──────────
def serialise(result, world_ok=True, width=W, height=H):
    dets = []
    for det in result.get("detections", []):
        dets.append({
            "id":     det["track_id"],
            "bbox":   det["bbox_xyxy"],
            "conf":   bool(det.get("track_confirmed", False)),
            "dist":   det.get("distance_to_agv_m"),
            "region": det.get("region_relative_to_agv"),
            "cam":    det.get("position_camera_m"),
            "world":  det.get("position_world_m"),
            "pred":   [[p["x"], p["y"]] for p in det.get("predictions_world", [])][::2],
        })
    agv = result.get("agv_movement", {})
    return {
        "stamp":       {"sec": 1234, "nsec": 567000000},
        "frame_id":    "zed_left_camera_optical_frame",
        "width":       width,
        "height":      height,
        "world_frame": "odom" if world_ok else None,
        "cmd":         (agv.get("command") or "normal_operation").upper(),
        "state":       agv.get("range_state"),
        "closest_m":   agv.get("closest_human_depth_m"),
        "latency_ms":  31.4,
        "detections":  dets,
    }


payload = serialise(result)

# json.dumps must not choke on numpy scalars — a classic silent killer here.
try:
    blob = json.dumps(payload, separators=(",", ":"))
    check("payload is JSON-serialisable (no numpy scalars leak through)", True,
          f"{len(blob)} bytes")
except TypeError as exc:
    check("payload is JSON-serialisable (no numpy scalars leak through)", False, str(exc))
    blob = None

if blob:
    check("payload stays small enough for 30 Hz", len(blob) < 8192, f"{len(blob)} bytes")
    check("no NaN/Infinity in the wire format",
          "NaN" not in blob and "Infinity" not in blob)


# ── Deserialise exactly as overlay_node._build_humans does ────────────────
def build_humans(det):
    out = []
    for d in det.get("detections", []):
        world = d.get("world") or None
        p = d.get("pred") or []
        out.append(R.Human(
            track_id=int(d.get("id", -1)),
            bbox=d.get("bbox", [0, 0, 0, 0]),
            distance_m=d.get("dist"),
            region=d.get("region"),
            confirmed=bool(d.get("conf", False)),
            world_xy=None if world is None else (float(world["x"]), float(world["y"])),
            prediction_xy=(np.asarray(p, dtype=np.float64).reshape(-1, 2)
                           if len(p) >= 2 else None),
        ))
    return out


rx = json.loads(blob)
humans = build_humans(rx)

check("round-trip preserves track count", len(humans) == len(result["detections"]),
      f"{len(humans)} vs {len(result['detections'])}")
check("bbox survives as 4 numbers", all(len(h.bbox) == 4 for h in humans))
check("world position survives", all(h.world_xy is not None for h in humans))
check("forecast survives as an (N,2) array",
      all(h.prediction_xy is not None and h.prediction_xy.shape[1] == 2 for h in humans),
      str([None if h.prediction_xy is None else h.prediction_xy.shape for h in humans]))
check("stamp round-trips to nanoseconds",
      rx["stamp"]["sec"] * 1_000_000_000 + rx["stamp"]["nsec"] == 1234_567000000)
check("cmd is upper-case and renderable", rx["cmd"] in R.C_STATE, rx["cmd"])


# ── The renderer must accept it ───────────────────────────────────────────
intr = R.Intrinsics(700.0, 700.0, 640.0, 360.0, 1280, 720)
img = np.zeros((720, 1280, 3), np.uint8)

# Bboxes are in source-image pixels; the overlay scales them to the render size.
sx, sy = 1280 / rx["width"], 720 / rx["height"]
for h in humans:
    h.bbox = [h.bbox[0] * sx, h.bbox[1] * sy, h.bbox[2] * sx, h.bbox[3] * sy]

scene = R.Scene(image=img, intrinsics=intr, T_opt_world=np.linalg.inv(T_wo),
                ground_z=0.0, humans=humans, cmd=rx["cmd"],
                closest_human_m=rx["closest_m"],
                plan_xy=np.array([[1.0, 0.0], [4.0, 0.5]]),
                travelled_xy=np.array([[-2.0, 0.0], [0.0, 0.0]]),
                robot_xy=(0.0, 0.0), robot_yaw=0.0, speed_mps=0.4)
try:
    out = R.render(scene)
    check("renderer accepts the deserialised payload",
          out.shape == (720, 1280, 3), str(out.shape))
    check("render actually drew something", out.sum() > 0)
except Exception as exc:
    check("renderer accepts the deserialised payload", False, f"{type(exc).__name__}: {exc}")


# ── Degraded payloads must not crash the renderer ─────────────────────────
def render_safe(det_payload, label):
    try:
        hs = build_humans(det_payload)
        sc = R.Scene(image=np.zeros((480, 640, 3), np.uint8), intrinsics=intr,
                     T_opt_world=None, humans=hs,
                     cmd=det_payload.get("cmd", "NORMAL_OPERATION"))
        R.render(sc)
        return True
    except Exception as exc:
        print(f"      {label}: {type(exc).__name__}: {exc}")
        return False


no_world = json.loads(json.dumps(serialise(result, world_ok=False)))
for d in no_world["detections"]:
    d["world"] = None
    d["pred"] = []
check("survives a payload with no world data (TF down)",
      render_safe(no_world, "no-world"))
check("survives an empty detection list",
      render_safe({"cmd": "STOP", "detections": []}, "empty"))
check("survives a single-point forecast",
      render_safe({"cmd": "SLOW_DOWN", "detections": [
          {"id": 1, "bbox": [10, 10, 50, 90], "conf": True, "dist": 2.0,
           "world": {"x": 1.0, "y": 0.0}, "pred": [[1.0, 0.0]]}]}, "1-pt"))
check("survives a missing-keys payload",
      render_safe({"detections": [{"id": 9, "bbox": [0, 0, 20, 40]}]}, "sparse"))
check("survives an unknown cmd string",
      render_safe({"cmd": "WAT", "detections": []}, "bad-cmd"))

# ── Report ────────────────────────────────────────────────────────────────
width = max(len(n) for n, _, _ in checks)
failed = 0
for name, ok, detail in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {name.ljust(width)}  {detail}")
    failed += (not ok)
print(f"\n{len(checks) - failed}/{len(checks)} passed")
sys.exit(1 if failed else 0)
