#!/usr/bin/env python3
"""
Check that a trial folder captured everything. Run automatically by
'./run_zed_e2e.sh stop'; run by hand on any trial:

    python3 tools/trial_check.py ~/g1_trials/20261026_141500_corridor_2ped

Reads only files (bag metadata.yaml, CSVs, snapshots), so it needs no ROS.
Exit code 1 if anything REQUIRED is missing, so it can gate a batch script.
"""
from __future__ import annotations

import csv
import glob
import json
import os
import re
import sys

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required (sudo apt install python3-yaml)")

# Without these the trial cannot support the latency / stopping / separation analysis.
REQUIRED = [
    "/g1/detections", "/g1/human_cmd", "/cmd_vel", "/odom", "/tf", "/tf_static",
    "/zed/zed_node/rgb/color/rect/camera_info", "/rosout",
]
REQUIRED_WITH_ROBOT = ["/g1/robot_cmd"]
# Present whenever a goal was sent. (Humble's RPP publishes received_global_plan
# and lookahead_point; nothing publishes /local_plan in this configuration.)
EXPECTED_WITH_GOAL = ["/plan", "/received_global_plan", "/lookahead_point",
                      "/behavior_tree_log", "/navigate_to_pose/_action/status"]

problems: list[str] = []
warnings: list[str] = []


def load_yaml(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return None


def check_bag(trial, with_robot, goal_sent):
    meta = load_yaml(os.path.join(trial, "bag", "metadata.yaml"))
    if meta is None:
        problems.append("bag/metadata.yaml missing: the recorder was not stopped with SIGINT. "
                        "Try: ros2 bag reindex <trial>/bag")
        return {}
    info = meta.get("rosbag2_bagfile_information", {})
    dur_s = info.get("duration", {}).get("nanoseconds", 0) / 1e9
    counts = {t["topic_metadata"]["name"]: t["message_count"]
              for t in info.get("topics_with_message_count", [])}

    try:
        with open(os.path.join(trial, "bag_topics.txt")) as f:
            requested = [ln.strip() for ln in f if ln.strip()]
    except OSError:
        requested = sorted(counts)

    required = REQUIRED + (REQUIRED_WITH_ROBOT if with_robot else [])
    expected = EXPECTED_WITH_GOAL if goal_sent else []
    print(f"\nRosbag: {dur_s:.1f} s, {info.get('message_count', 0)} messages")
    print(f"  {'topic':48s} {'msgs':>8s} {'Hz':>7s}")
    for t in requested + sorted(set(counts) - set(requested)):
        n = counts.get(t, 0)
        hz = n / dur_s if dur_s > 0 else 0.0
        tag = ""
        if n == 0 and t in required:
            tag = "  <-- REQUIRED, MISSING"
            problems.append(f"no messages on required topic {t}")
        elif n == 0 and t in expected:
            tag = "  <-- expected (goal was sent)"
            warnings.append(f"no messages on {t} although a goal was sent")
        elif n == 0:
            tag = "  (optional, absent)"
        print(f"  {t:48s} {n:8d} {hz:7.1f}{tag}")

    cam, det = counts.get("/zed/zed_node/rgb/color/rect/camera_info", 0), counts.get("/g1/detections", 0)
    if cam and det:
        print(f"\n  detection_node processed {det / cam:.0%} of camera frames "
              f"({det / dur_s:.1f} of {cam / dur_s:.1f} Hz)")
    return counts


def csv_stats(path):
    """(rows, columns, span_s) of a CSV whose first column is t_host_ns."""
    rows, cols, first, last = 0, 0, None, None
    with open(path, newline="") as f:
        r = csv.reader(f)
        header = next(r, None) or []
        cols = len(header)
        for row in r:
            rows += 1
            if first is None:
                first = row[0]
            last = row[0]
    try:
        span = (int(last) - int(first)) / 1e9 if rows > 1 else 0.0
    except (TypeError, ValueError):
        span = 0.0
    return rows, cols, span


def report_csv(label, path, required, missing_note=""):
    """Print one CSV's rows and rate; record a problem/warning if it is empty."""
    if not os.path.exists(path):
        (problems if required else warnings).append(f"{label} missing")
        print(f"  {label:34s} missing")
        return
    rows, cols, span = csv_stats(path)
    rate = f"{rows / span:.0f} Hz" if span > 0 else "-"
    print(f"  {label:34s} {rows:8d} rows  {cols:4d} cols  {rate:>7s}"
          + (f"  ({missing_note})" if rows == 0 and missing_note else ""))
    if rows == 0 and required:
        problems.append(f"{label} has no data rows")


def check_robot_state(trial, with_robot):
    d = os.path.join(trial, "robot_state")
    print("\nRobot state (onboard, via unitree_sdk2py):")
    if not os.path.isdir(d):
        (problems if with_robot else warnings).append("robot_state/ missing")
        print("  none recorded")
        return
    report_csv("robot_state/lowstate.csv", os.path.join(d, "lowstate.csv"), with_robot)
    report_csv("robot_state/motor_health.csv", os.path.join(d, "motor_health.csv"), with_robot)
    report_csv("robot_state/secondary_imu.csv", os.path.join(d, "secondary_imu.csv"), False,
               "torso IMU not published?")
    known = {"lowstate.csv", "motor_health.csv", "secondary_imu.csv"}
    for p in sorted(glob.glob(os.path.join(d, "*.csv"))):
        if os.path.basename(p) not in known:
            report_csv(f"robot_state/{os.path.basename(p)}", p, False,
                       "firmware does not publish it")


def check_system_and_svo(trial, want_svo):
    print("\nComputer load and camera recording:")
    report_csv("system.csv", os.path.join(trial, "system.csv"), True)
    svo = os.path.join(trial, "camera.svo2")
    if not want_svo:
        print("  camera.svo2                        (SVO=0)")
        return
    size = os.path.getsize(svo) if os.path.exists(svo) else 0
    print(f"  camera.svo2                        {size / 1e6:8.1f} MB")
    if size == 0:
        log = os.path.join(trial, "svo.log")
        why = ""
        if os.path.exists(log):
            with open(log) as f:
                m = re.search(r"message='([^']*)'", f.read())
            why = f" ({m.group(1)})" if m else " (see svo.log)"
        problems.append(f"SVO recording requested but camera.svo2 is missing or empty{why}")


def check_files(trial, record_video):
    print("\nFiles:")
    mp4 = os.path.join(trial, "overlay.mp4")
    if record_video:
        size = os.path.getsize(mp4) if os.path.exists(mp4) else 0
        print(f"  overlay.mp4            {size / 1e6:.1f} MB")
        if size == 0:
            warnings.append("overlay.mp4 missing or empty")

    logs = glob.glob(os.path.join(trial, "logs", "*.log"))
    print(f"  logs/                  {len(logs)} pane logs")
    ros_logs = glob.glob(os.path.join(trial, "ros_logs", "**", "*"), recursive=True)
    print(f"  ros_logs/              {len([p for p in ros_logs if os.path.isfile(p)])} files")

    params = glob.glob(os.path.join(trial, "snapshot", "params", "*.yaml"))
    print(f"  snapshot/params/       {len(params)} node parameter dumps")
    if not params:
        warnings.append("no parameter snapshot (did the trial run ~40 s before stopping?)")

    gv = glob.glob(os.path.join(trial, "snapshot", "tf", "frames*.gv"))
    if gv:
        with open(gv[0]) as f:
            edges = re.findall(r'"([^"]+)"\s*->\s*"([^"]+)"', f.read())
        frames = {name for edge in edges for name in edge}
        print(f"  snapshot/tf/           {len(frames)} frames; map frame present: "
              f"{'YES' if 'map' in frames else 'no'}")
    else:
        warnings.append("no TF tree snapshot (snapshot/tf/frames*.gv)")

    echo = os.path.join(trial, "snapshot", "tf", "odom_to_pelvis.txt")
    if os.path.exists(echo):
        with open(echo) as f:
            m = re.search(r"Translation:\s*\[([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\]", f.read())
        if m:
            print(f"  odom->pelvis at snapshot: z = {float(m.group(3)):+.3f} m")

    stale = os.path.join(trial, "code", "install_vs_src.txt")
    if os.path.exists(stale):
        with open(stale) as f:
            bad = [ln.strip() for ln in f if ln.startswith(("STALE", "MISSING"))]
        for b in bad:
            problems.append(f"build mismatch: {b}")
        print(f"  code/                  install vs src: {'MISMATCH' if bad else 'match'}")

    events = os.path.join(trial, "events.jsonl")
    n_ev = 0
    if os.path.exists(events):
        with open(events) as f:
            n_ev = sum(1 for _ in f)
    print(f"  events.jsonl           {n_ev} events")

    meta = load_yaml(os.path.join(trial, "trial.yaml")) or {}
    empty = [k for k, v in meta.items() if v in ("", None)]
    if empty:
        warnings.append("trial.yaml fields still empty: " + ", ".join(empty))


def main() -> int:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    trial = os.path.abspath(os.path.expanduser(sys.argv[1]))
    if not os.path.isdir(trial):
        sys.exit(f"not a directory: {trial}")

    launch = load_yaml(os.path.join(trial, "launch_info.yaml")) or {}
    with_robot = str(launch.get("with_robot", "0")) == "1"
    record_video = str(launch.get("overlay", "1")) == "1" and str(launch.get("record_video", "1")) == "1"
    goal_sent = False
    ev_path = os.path.join(trial, "events.jsonl")
    if os.path.exists(ev_path):
        with open(ev_path) as f:
            goal_sent = any(json.loads(ln).get("kind") == "goal_sent" for ln in f if ln.strip())

    print(f"Trial {trial}")
    print(f"  with_robot={int(with_robot)}  use_zed_odom={launch.get('use_zed_odom')}  "
          f"git={str(launch.get('git_commit', '?'))[:10]}  dirty={launch.get('git_dirty')}")
    check_bag(trial, with_robot, goal_sent)
    check_robot_state(trial, with_robot)
    check_system_and_svo(trial, str(launch.get("svo", "0")) == "1")
    check_files(trial, record_video)

    print()
    for w in warnings:
        print(f"WARN  {w}")
    for p in problems:
        print(f"FAIL  {p}")
    print("OK - trial is complete." if not problems else
          f"{len(problems)} required item(s) missing.")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
