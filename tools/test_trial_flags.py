#!/usr/bin/env python3
"""
Checks for the pure behaviour checks in tools/trial_flags.py, on synthetic
records shaped like the 2026-10-27 problems. Runs without ROS:
`python3 tools/test_trial_flags.py`.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(__file__))
import trial_flags as tf  # noqa: E402

checks = []
S = 10**9


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))


# /odom: smooth walk, then a 1.75 m reset (trial 180825 at 62 s)
odom = [{"t": i * S // 50, "x": i * 0.01, "y": 0.0} for i in range(100)]
odom.append({"t": 100 * S // 50, "x": odom[-1]["x"] + 1.75, "y": 0.0})
j = tf.odom_jumps(odom)
check("one odometry jump found, size 1.75 m", len(j) == 1 and abs(j[0][1] - 1.75) < 1e-6, j)
check("normal walking is not a jump", tf.odom_jumps(odom[:100]) == [])

# robot_cmd: a 10 s SDK call, a failed call, normal calls
rc = [{"event": "cmd_vel", "t_call": 0, "t": int(0.002 * S), "sent": [0.5, 0, 0], "ok": True, "ret": 0},
      {"event": "cmd_vel", "t_call": 1 * S, "t": 11 * S, "sent": [0.5, 0, 0], "ok": True, "ret": None},
      {"event": "stop_move", "t_call": 12 * S, "t": int(12.6 * S), "ok": False, "ret": 3104},
      {"event": "cmd_vel", "vin": [0.4, 0, 0], "blocked": True, "t": 13 * S}]
slow = tf.slow_calls(rc)
check("10 s and 0.6 s calls flagged as slow", [round(d, 1) for _, _, d, _ in slow] == [10.0, 0.6], slow)
check("failed call (code 3104) flagged", [c for _, _, c in tf.failed_calls(rc)] == [3104])

# goals: one aborted at once, then the robot keeps getting walking commands
status = [{"t": 1 * S, "goals": [("aaaa", 2)]}, {"t": int(1.1 * S), "goals": [("aaaa", 6)]},
          {"t": 30 * S, "goals": [("aaaa", 6), ("bbbb", 2)]}, {"t": 40 * S, "goals": [("aaaa", 6), ("bbbb", 4)]}]
goals = tf.goal_timeline(status)
check("goal outcomes in order", [(g[0], g[3]) for g in goals] == [("aaaa", 6), ("bbbb", 4)], goals)
walk = [{"event": "cmd_vel", "t": t * S, "sent": [0.3, 0.0, 0.0]} for t in (2, 5, 13)]
orphan = tf.walking_after_goal_end(goals, walk)
check("walking 12 s after an ABORTED goal flagged", len(orphan) == 1 and orphan[0][0] == "aaaa"
      and round(orphan[0][2], 1) == 11.9 and orphan[0][3] == 2, orphan)
check("zero velocities after a goal are not flagged",
      tf.walking_after_goal_end(goals, [{"event": "cmd_vel", "t": 5 * S, "sent": [0.0, 0.0, 0.0]}]) == [])
check("commands within the next goal are not flagged",
      tf.walking_after_goal_end(goals, [{"event": "cmd_vel", "t": 35 * S, "sent": [0.3, 0, 0]}]) == [])

# STOP flicker
det = [{"t": int(t * S), "cmd": c} for t, c in
       [(0, "NORMAL_OPERATION"), (1, "STOP"), (1.2, "NORMAL_OPERATION"), (1.4, "STOP"), (5, "NORMAL_OPERATION")]]
check("STOP -> NORMAL -> STOP within 1 s flagged once", len(tf.stop_flicker(det)) == 1)

logs = [{"msg": "Timed out while waiting for action server to acknowledge goal request for follow_path"},
        {"msg": "Control loop missed its desired rate of 10.0000 Hz. Current loop rate is 3.3296 Hz."}]
c = tf.nav2_log_counts(logs)
check("Nav2 timeout and missed-rate messages counted",
      c["action server timeout"] == 1 and c["control loop missed rate"] == 1, c)

# hand-parsed GoalStatusArray (bags whose hidden action topics have no schema)
body = struct.pack("<I", 2)                # CDR: offsets count from after the 4-byte header
for uid, st in ((bytes(range(16)), 2), (bytes(range(16, 32)), 6)):
    body += uid                                          # uint8[16], no alignment
    body += b"\x00" * (-len(body) % 4)                   # Time is 4-byte aligned
    body += struct.pack("<iI", 5, 7) + bytes([st])
buf = b"\x00\x01\x00\x00" + body
check("GoalStatusArray CDR parsed", tf._parse_goal_status_cdr(buf) ==
      [(bytes(range(16)).hex()[:8], 2), (bytes(range(16, 32)).hex()[:8], 6)], tf._parse_goal_status_cdr(buf))

w = max(len(n) for n, _, _ in checks)
for name, ok, detail in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {name:{w}s}  {detail if not ok else ''}")
n_ok = sum(ok for _, ok, _ in checks)
print(f"\n{n_ok}/{len(checks)} passed")
sys.exit(0 if n_ok == len(checks) else 1)
