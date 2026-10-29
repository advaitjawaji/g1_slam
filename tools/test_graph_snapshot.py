#!/usr/bin/env python3
"""
Checks for the pure helpers in tools/graph_snapshot.py, and that trial_check.py
still reads the TF files it writes. Runs without ROS:
`python3 tools/test_graph_snapshot.py`.
"""
import os
import re
import sys
import types

sys.path.insert(0, os.path.dirname(__file__))
import graph_snapshot as gs  # noqa: E402

checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))


def pv(t, **kw):
    v = types.SimpleNamespace(bool_value=False, integer_value=0, double_value=0.0, string_value="",
                              byte_array_value=[], bool_array_value=[], integer_array_value=[],
                              double_array_value=[], string_array_value=[], type=t)
    v.__dict__.update(kw)
    return v


check("bool", gs.param_value(pv(1, bool_value=True)) is True)
check("integer", gs.param_value(pv(2, integer_value=200)) == 200)
check("double", gs.param_value(pv(3, double_value=0.25)) == 0.25)
check("string", gs.param_value(pv(4, string_value="odom")) == "odom")
check("double array -> list", gs.param_value(pv(8, double_array_value=(0.5, 0.0, 2.5))) == [0.5, 0.0, 2.5])
check("NOT_SET -> None", gs.param_value(pv(0)) is None)

nested = gs.nest({"pos_tracking.reset_odom_with_loop_closure": False,
                  "pos_tracking.area_memory": True, "use_sim_time": False})
check("dotted names nest like ros2 param dump",
      nested == {"pos_tracking": {"area_memory": True, "reset_odom_with_loop_closure": False},
                 "use_sim_time": False}, nested)

check("file name for a namespaced node", gs.file_name("/zed/zed_node") == "zed_zed_node.yaml")

frames = {"zed_camera_link": {"parent": "odom", "broadcaster": "zed_node", "rate": 30.0},
          "pelvis": {"parent": "zed_camera_link", "broadcaster": "static", "rate": 10000.0}}
gv = gs.frames_gv(frames)
edges = re.findall(r'"([^"]+)"\s*->\s*"([^"]+)"', gv)       # trial_check.py's regex
check("trial_check reads the .gv edges", sorted(edges) == [("odom", "zed_camera_link"),
                                                          ("zed_camera_link", "pelvis")], edges)
echo = "Translation: [0.012, -0.003, -0.401]\nRotation: in Quaternion [0.0, 0.1, 0.0, 0.99]\n"
m = re.search(r"Translation:\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]", echo)  # trial_check.py's regex
check("trial_check reads odom->pelvis z", m and float(m.group(3)) == -0.401)

w = max(len(n) for n, _, _ in checks)
for name, ok, detail in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {name:{w}s}  {detail if not ok else ''}")
n_ok = sum(ok for _, ok, _ in checks)
print(f"\n{n_ok}/{len(checks)} passed")
sys.exit(0 if n_ok == len(checks) else 1)
