#!/usr/bin/env python3
"""
Behaviour flags for one trial, read from its rosbag. Called by trial_check.py
so problems show up at the end of the trial that had them, not weeks later:

  - /odom jumps (ZED VIO reset) and ZED loop closures
  - SDK calls that blocked (robot_node could send nothing meanwhile) or failed
  - goal outcomes, and robot commanded to walk after its goal had ended
  - STOP released then re-issued within 1 s (flicker)
  - Nav2 timeouts and missed control rates in /rosout

Each check below was a real problem in the 2026-10-27 trials.

    python3 tools/trial_flags.py <trial_dir>

Reads the bag with rosbag2_py (ROS 2 sourced) or, off the robot, with the
`mcap` + `mcap-ros2-support` pip packages.
"""
from __future__ import annotations

import glob
import json
import os
import struct
import sys

ODOM_JUMP_M = 0.15          # step between consecutive /odom samples
ODOM_JUMP_DT_S = 0.2        # ... at most this far apart
SLOW_CALL_S = 0.3           # an SDK call longer than this blocked robot_node
GOAL_END_GRACE_S = 2.0      # velocity commands this long after a goal ended
FLICKER_S = 1.0
STATUS = {1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
TERMINAL = {4, 5, 6}
NAV2_LOG_PATTERNS = {
    "action server timeout": "Timed out while waiting for action server",
    "control loop missed rate": "Control loop missed its desired rate",
    "failed to make progress": "Failed to make progress",
}
TOPICS = ["/odom", "/zed/zed_node/pose/status", "/g1/robot_cmd", "/g1/detections",
          "/navigate_to_pose/_action/status", "/rosout"]


# ── pure checks (records are plain dicts; times in ns) ──────────────────────
def odom_jumps(odom):
    out = []
    for a, b in zip(odom, odom[1:]):
        dt = (b["t"] - a["t"]) / 1e9
        step = ((b["x"] - a["x"]) ** 2 + (b["y"] - a["y"]) ** 2) ** 0.5
        if step > ODOM_JUMP_M and dt < ODOM_JUMP_DT_S:
            out.append((b["t"], step))
    return out


def slow_calls(robot_cmd):
    return [(r["t"], r["event"], (r["t"] - r["t_call"]) / 1e9, r.get("ret"))
            for r in robot_cmd if "t_call" in r and (r["t"] - r["t_call"]) / 1e9 > SLOW_CALL_S]


def failed_calls(robot_cmd):
    return [(r["t"], r["event"], r.get("ret")) for r in robot_cmd
            if r["event"] in ("cmd_vel", "stop_move") and not r.get("blocked")
            and r.get("ok") is False]


def goal_timeline(status_msgs):
    """[(goal, t_first, t_end or None, final_status)] in order of appearance."""
    goals = {}
    for m in status_msgs:
        for gid, st in m["goals"]:
            g = goals.setdefault(gid, {"t0": m["t"], "t_end": None, "st": st})
            g["st"] = st
            if st in TERMINAL and g["t_end"] is None:
                g["t_end"] = m["t"]
    return sorted(((k, g["t0"], g["t_end"], g["st"]) for k, g in goals.items()), key=lambda x: x[1])


def walking_after_goal_end(goals, robot_cmd):
    """Nonzero velocities sent to the robot > GOAL_END_GRACE_S after a goal ended
    and before the next goal began: something other than the navigator drove it."""
    starts = [g[1] for g in goals]
    out = []
    for i, (gid, t0, t_end, st) in enumerate(goals):
        if t_end is None:
            continue
        nxt = min([s for s in starts if s > t_end], default=float("inf"))
        moving = [r["t"] for r in robot_cmd
                  if r["event"] == "cmd_vel" and r.get("sent")
                  and any(abs(v) > 1e-3 for v in r["sent"])
                  and t_end + GOAL_END_GRACE_S * 1e9 < r["t"] < nxt]
        if moving:
            out.append((gid, STATUS.get(st, st), (moving[-1] - t_end) / 1e9, len(moving)))
    return out


def stop_flicker(detections):
    """STOP -> NORMAL -> STOP where STOP returns within FLICKER_S."""
    segs = []
    for d in detections:
        if not segs or segs[-1][0] != d["cmd"]:
            segs.append([d["cmd"], d["t"], d["t"]])
        segs[-1][2] = d["t"]
    return [s[1] for p, s, n in zip(segs, segs[1:], segs[2:])
            if p[0] == "STOP" and s[0] == "NORMAL_OPERATION" and n[0] == "STOP"
            and (n[1] - p[2]) / 1e9 < FLICKER_S]


def nav2_log_counts(rosout):
    return {k: sum(1 for r in rosout if pat in r["msg"]) for k, pat in NAV2_LOG_PATTERNS.items()}


def analyze_records(rec, t0):
    """-> (report lines, warnings)."""
    S = lambda ns: f"{(ns - t0) / 1e9:.1f}s"
    lines, warns = [], []

    jumps = odom_jumps(rec["/odom"])
    closures = sum(1 for r in rec["/zed/zed_node/pose/status"] if r["spatial"] == 1)
    lines.append(f"  /odom jumps > {ODOM_JUMP_M} m: {len(jumps)}"
                 + (f" ({', '.join(f'{d:.2f} m @{S(t)}' for t, d in jumps[:5])})" if jumps else "")
                 + f"; ZED loop closures: {closures}")
    if jumps:
        warns.append(f"/odom jumped {len(jumps)}x (largest {max(d for _, d in jumps):.2f} m): "
                     "odometry-based metrics need care in this trial")

    slow, failed = slow_calls(rec["/g1/robot_cmd"]), failed_calls(rec["/g1/robot_cmd"])
    lines.append(f"  SDK calls > {SLOW_CALL_S} s: {len(slow)}"
                 + (f" ({', '.join(f'{ev} {d:.1f} s @{S(t)}' for t, ev, d, _ in slow[:5])})" if slow else "")
                 + f"; failed calls: {len(failed)}"
                 + (f" (codes {sorted({str(c) for _, _, c in failed})})" if failed else ""))
    if slow:
        warns.append(f"{len(slow)} SDK call(s) blocked robot_node > {SLOW_CALL_S} s "
                     f"(longest {max(d for _, _, d, _ in slow):.1f} s)")

    goals = goal_timeline(rec["/navigate_to_pose/_action/status"])
    lines.append(f"  goals: {len(goals)}" + (" (" + ", ".join(
        f"{STATUS.get(st, st)}" + (f" after {(te - t0g) / 1e9:.1f} s" if te else " (no result)")
        for _, t0g, te, st in goals) + ")" if goals else ""))
    for gid, st, dt, n in walking_after_goal_end(goals, rec["/g1/robot_cmd"]):
        warns.append(f"robot sent {n} walking command(s) up to {dt:.1f} s after goal {gid} ended "
                     f"({st}): not under the navigator's control")
    aborted = [g for g in goals if g[3] == 6]
    if aborted:
        warns.append(f"{len(aborted)} of {len(goals)} goal(s) ABORTED")

    flick = stop_flicker(rec["/g1/detections"])
    n_stop = sum(1 for a, b in zip(rec["/g1/detections"], rec["/g1/detections"][1:])
                 if b["cmd"] == "STOP" and a["cmd"] != "STOP")
    lines.append(f"  STOP onsets: {n_stop}; STOP->NORMAL->STOP within {FLICKER_S:.0f} s: {len(flick)}")
    if flick:
        warns.append(f"STOP flickered {len(flick)}x (released and re-issued within {FLICKER_S:.0f} s)")

    logs = nav2_log_counts(rec["/rosout"])
    lines.append("  Nav2 log: " + ", ".join(f"{k} {v}" for k, v in logs.items()))
    for k, v in logs.items():
        if v and k != "failed to make progress":
            warns.append(f"Nav2 {k}: {v}x in /rosout")
    return lines, warns


# ── bag reading ─────────────────────────────────────────────────────────────
def _uuid8(u):
    return bytes(bytearray(int(x) for x in u)).hex()[:8]


def _record(topic, t, msg):
    if topic == "/odom":
        p = msg.pose.pose.position
        return {"t": t, "x": p.x, "y": p.y}
    if topic == "/zed/zed_node/pose/status":
        return {"t": t, "spatial": int(msg.spatial_memory_status)}
    if topic == "/g1/robot_cmd":
        return json.loads(msg.data)
    if topic == "/g1/detections":
        d = json.loads(msg.data)
        stamp = d["stamp"]["sec"] * 10**9 + d["stamp"]["nsec"]
        return {"t": stamp, "cmd": d.get("cmd"), "n": len(d.get("detections", []))}
    if topic == "/navigate_to_pose/_action/status":
        return {"t": t, "goals": [(_uuid8(s.goal_info.goal_id.uuid), int(s.status)) for s in msg.status_list]}
    if topic == "/rosout":
        return {"t": t, "level": int(msg.level), "name": msg.name, "msg": msg.msg}
    return None


def _parse_goal_status_cdr(buf):
    # action_msgs/GoalStatusArray in little-endian CDR, for bags whose hidden
    # action topics carry no schema (rosbag2 on Jazzy).
    b, out = buf[4:], []                                    # skip encapsulation header
    n, = struct.unpack_from("<I", b, 0)
    o = 4
    for _ in range(n):
        gid = b[o:o + 16].hex()[:8]; o += 16
        o = (o + 3) // 4 * 4; o += 8                       # builtin_interfaces/Time
        out.append((gid, b[o])); o += 1
    return out


def read_bag(trial):
    bag_dir = os.path.join(trial, "bag")
    rec = {t: [] for t in TOPICS}
    t_first = None
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        rosbag2_py = None
    if rosbag2_py is not None:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                    rosbag2_py.ConverterOptions("", ""))
        types = {m.name: m.type for m in reader.get_all_topics_and_types()}
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
        while reader.has_next():
            topic, data, t = reader.read_next()
            t_first = t if t_first is None else min(t_first, t)
            r = _record(topic, t, deserialize_message(data, get_message(types[topic])))
            if r is not None:
                rec[topic].append(r)
        return rec, t_first
    from mcap.reader import make_reader                    # off-robot fallback
    from mcap_ros2.decoder import DecoderFactory
    path = sorted(glob.glob(os.path.join(bag_dir, "*.mcap")))[0]
    status = "/navigate_to_pose/_action/status"
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        t_first = reader.get_summary().statistics.message_start_time
        for _, ch, m, msg in reader.iter_decoded_messages(topics=[t for t in TOPICS if t != status]):
            r = _record(ch.topic, m.log_time, msg)
            if r is not None:
                rec[ch.topic].append(r)
    with open(path, "rb") as f:
        for _, ch, m in make_reader(f).iter_messages(topics=[status]):
            rec[status].append({"t": m.log_time, "goals": _parse_goal_status_cdr(m.data)})
    return rec, t_first


def analyze(trial):
    rec, t0 = read_bag(trial)
    return analyze_records(rec, t0 or 0)


def main() -> int:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    lines, warns = analyze(sys.argv[1])
    print("\n".join(lines))
    for w in warns:
        print(f"WARN  {w}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
