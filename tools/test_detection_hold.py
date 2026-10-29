#!/usr/bin/env python3
"""
Checks for the STOP/SLOW hold in `HumanXZPredictor`.

STOP and SLOW are held for `agv_hold_max_frames` processed frames. Frames on
which YOLO finds nobody must count down that hold like any other frame; before
this fix they returned NORMAL_OPERATION immediately, so a person stepping out
of the camera's view released a STOP at once (seen in trial
20261027_180412_robot_closed_lab_trial_human_headon at 47.27 s).

Runs without ROS, YOLO or a camera: `python3 tools/test_detection_hold.py`.
"""
import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "g1_detection"))

_ul = types.ModuleType("ultralytics")
_ul.YOLO = lambda *a, **k: object()
sys.modules.setdefault("ultralytics", _ul)

from g1_detection.detection import HumanXZPredictor  # noqa: E402

checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))


H, W = 120, 160
BOX = np.array([60.0, 20.0, 100.0, 110.0], dtype=np.float32)


def make_predictor():
    p = HumanXZPredictor(model_path="stub", draw=False, profile=True, detect_every_n=1)
    p.model = None
    p._run_model = lambda frame: ["stub"]
    return p


def run(p, frames):
    """frames: list of depth (m) of one person straight ahead, or None = nobody seen."""
    out = []
    color = np.zeros((H, W, 3), np.uint8)
    for z in frames:
        cloud = np.zeros((H, W, 3), np.float32)
        if z is None:
            p._parse_yolo_results = lambda r: []
            cloud[..., 2] = 5.0
        else:
            p._parse_yolo_results = lambda r: [{"track_id": 1, "box": BOX}]
            cloud[..., 2] = z
        _, res = p.process_frame(color, cloud, timestamp=len(out) * 0.15)
        out.append(res["agv_movement"]["command"])
    return out


hold = make_predictor().agv_hold_max_frames

# 1. STOP survives the person leaving view for the length of the hold, then clears.
cmds = run(make_predictor(), [1.5] * 6 + [None] * (hold + 3))
first_stop = cmds.index("stop") if "stop" in cmds else None
check("person at 1.5 m ahead triggers STOP", first_stop is not None, cmds[:6])
gone = cmds[6:]
check(f"STOP held for {hold} empty frames after the person leaves view",
      gone[:hold] == ["stop"] * hold, gone[:hold])
check("STOP clears once the hold has run out", gone[hold:] == ["normal_operation"] * 3, gone[hold:])

# 2. No flicker: one empty frame between two STOP frames keeps STOP throughout.
cmds = run(make_predictor(), [1.5] * 6 + [None] + [1.5] * 3)
check("a single empty frame does not interrupt STOP",
      cmds[first_stop:] == ["stop"] * (len(cmds) - first_stop), cmds)

# 3. SLOW is held through empty frames too.
cmds = run(make_predictor(), [3.0] * 6 + [None] * 4)
check("person at 3.0 m ahead triggers SLOW", "slow_down" in cmds[:6], cmds[:6])
check("SLOW held through empty frames", cmds[6:] == ["slow_down"] * 4, cmds[6:])

# 4. Nobody ever seen: every frame is NORMAL, as before the fix.
cmds = run(make_predictor(), [None] * 5)
check("no person at all gives NORMAL on every frame", cmds == ["normal_operation"] * 5, cmds)

# 5. The empty-frame output carries the held state, not a fresh "clear" state.
p = make_predictor()
run(p, [1.5] * 6)
p._parse_yolo_results = lambda r: []
_, res = p.process_frame(np.zeros((H, W, 3), np.uint8), np.full((H, W, 3), 5.0, np.float32), 1.0)
check("empty frame during a hold reports the held range_state",
      res["agv_movement"]["range_state"] != "no_human_or_clear", res["agv_movement"])
check("empty frame still reports timings_ms when profiling", "timings_ms" in res, sorted(res))

w = max(len(n) for n, _, _ in checks)
for name, ok, detail in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {name:{w}s}  {detail if not ok else ''}")
n_ok = sum(ok for _, ok, _ in checks)
print(f"\n{n_ok}/{len(checks)} passed")
sys.exit(0 if n_ok == len(checks) else 1)
