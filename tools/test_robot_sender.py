#!/usr/bin/env python3
"""
Checks for robot_node's command delivery (sender thread + short SDK timeout).

  1. In normal operation the robot receives exactly the calls the previous
     robot_node sent: one SetVelocity per /cmd_vel (what Move() sends), one
     SetVelocity(0, 0, 0) per STOP (what StopMove() sends), same clamping/scaling.
  2. When an SDK call hangs, callbacks stay responsive, a STOP is sent as soon
     as the hung call times out, and no velocity received before the STOP is
     sent after it. (Trials 20261027_173238 and _180825 froze robot_node 10 s.)

Runs without ROS or the Unitree SDK: `python3 tools/test_robot_sender.py`.
"""
import json
import os
import sys
import threading
import time
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "g1_robot"))

checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))


def mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


# ── ROS 2 stand-ins ───────────────────────────────────────────────────────
PARAMS = {}        # parameter overrides for the next node
LOG = []           # /g1/robot_cmd records


class Pub:
    def __init__(self, topic): self.topic = topic
    def publish(self, msg):
        if self.topic == "/g1/robot_cmd":
            LOG.append(json.loads(msg.data))


class Logger:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass
    def error(self, *a, **k): pass


class Node:
    def __init__(self, name): self._params = {}
    def declare_parameter(self, n, v): self._params[n] = PARAMS.get(n, v)
    def get_parameter(self, n): return types.SimpleNamespace(value=self._params[n])
    def create_subscription(self, *a, **k): return None
    def create_publisher(self, typ, topic, qos): return Pub(topic)
    def get_logger(self): return Logger()
    def get_clock(self): return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=time.monotonic_ns()))
    def destroy_node(self): pass


class String:
    def __init__(self, data=""): self.data = data


class Twist:
    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.linear = types.SimpleNamespace(x=vx, y=vy, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=wz)


mod("rclpy", ok=lambda: True)
mod("rclpy.node", Node=Node)
mod("std_msgs"); mod("std_msgs.msg", String=String)
mod("geometry_msgs"); mod("geometry_msgs.msg", Twist=Twist)

# ── Unitree SDK stand-in: SetVelocity/Move/StopMove as in unitree_sdk2py ──
CALLS = []         # (t_monotonic, args)
HANG = {"n": 0}    # the next n SetVelocity calls get no reply


class LocoClient:
    def __init__(self): self.timeout = 1.0
    def SetTimeout(self, t): self.timeout = t
    def Init(self): pass

    def SetVelocity(self, vx, vy, omega, duration=1.0):
        if HANG["n"] > 0:
            HANG["n"] -= 1
            time.sleep(2 * self.timeout)          # SendRequest + GetResult both wait
            CALLS.append((time.monotonic(), ("timeout", round(vx, 6), round(vy, 6), round(omega, 6), duration)))
            return 3104                           # RPC_ERR_CLIENT_API_TIMEOUT
        CALLS.append((time.monotonic(), (round(vx, 6), round(vy, 6), round(omega, 6), duration)))
        return 0

    def Move(self, vx, vy, vyaw, continous_move=False):
        self.SetVelocity(vx, vy, vyaw, 864000.0 if continous_move else 1)

    def StopMove(self):
        self.SetVelocity(0., 0., 0.)


mod("unitree_sdk2py"); mod("unitree_sdk2py.core")
mod("unitree_sdk2py.core.channel", ChannelFactory=lambda: types.SimpleNamespace(Init=lambda *a: None))
mod("unitree_sdk2py.g1"); mod("unitree_sdk2py.g1.loco")
mod("unitree_sdk2py.g1.loco.g1_loco_client", LocoClient=LocoClient)

import g1_robot.g1 as g1mod  # noqa: E402
g1mod.time = types.SimpleNamespace(sleep=lambda s: None)   # skip the 1 s DDS settle
from g1_robot.robot_node import RobotNode  # noqa: E402


def drain(node, timeout=2.0):
    """Wait until the sender has nothing queued and is idle."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        with node._cv:
            idle = not node._stop_pending and node._pending_vin is None
        if idle:
            time.sleep(0.03)
            with node._cv:
                if not node._stop_pending and node._pending_vin is None:
                    return True
        time.sleep(0.005)
    return False


def new_node(**params):
    PARAMS.clear(); PARAMS.update(params)
    CALLS.clear(); LOG.clear(); HANG["n"] = 0
    return RobotNode()


h = lambda node, s: node._human_cmd_cb(String(s))
v = lambda node, *a: node._cmd_vel_cb(Twist(*a))

# 1. Normal operation: same calls as the previous robot_node -------------------
node = new_node()
script = [
    lambda: v(node, 0.3, 0.0, 0.1), lambda: h(node, "SLOW_DOWN"), lambda: v(node, 0.5, 0.1, -0.2),
    lambda: h(node, "STOP"), lambda: v(node, 0.4, 0.0, 0.0), lambda: h(node, "STOP"),
    lambda: h(node, "NORMAL_OPERATION"), lambda: v(node, 1.2, 0.9, 1.5), lambda: v(node, -1.0, -0.6, -2.0),
    lambda: v(node, 0.0, 0.0, 0.0), lambda: h(node, "garbage"), lambda: h(node, "slow_down"),
    lambda: v(node, 0.2, 0.0, 0.0), lambda: h(node, "normal_operation"), lambda: v(node, 0.25, 0.0, 0.0),
]
for step in script:
    step()
    drain(node)
node.destroy_node()
# What the previous robot_node sent for this script: Move(...) = SetVelocity(..., 1),
# StopMove() = SetVelocity(0, 0, 0), plus StopMove() on shutdown.
expected = [(0.3, 0.0, 0.1, 1), (0.2, 0.04, -0.08, 1), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0),
            (0.8, 0.4, 0.8, 1), (-0.8, -0.4, -0.8, 1), (0.0, 0.0, 0.0, 1), (0.08, 0.0, 0.0, 1),
            (0.25, 0.0, 0.0, 1), (0.0, 0.0, 0.0, 1.0)]
got = [c for _, c in CALLS]
check("normal operation sends the same SDK calls as before", got == expected, got)
check("sender thread exits on shutdown", not node._sender.is_alive())
ev = [r["event"] for r in LOG]
check("log records the blocked /cmd_vel during STOP",
      any(r["event"] == "cmd_vel" and r.get("blocked") for r in LOG), ev)
check("log carries the reply code (0) instead of null",
      all(r.get("ret") == 0 for r in LOG if r["event"] in ("cmd_vel", "stop_move") and not r.get("blocked")))

# 2. A hung SDK call: STOP still gets through promptly -------------------------
node = new_node(sdk_timeout_s=0.1)
HANG["n"] = 1
v(node, 0.5, 0.0, 0.0)                       # this call hangs for 0.2 s
time.sleep(0.03)
t0 = time.monotonic()
h(node, "STOP")
t_cb = time.monotonic() - t0
v(node, 0.5, 0.0, 0.0)                       # arrives during the STOP: must be blocked
drain(node)
check("STOP callback returns at once while an SDK call hangs", t_cb < 0.02, f"{t_cb * 1e3:.1f} ms")
stop_t = next((t for t, c in CALLS if c == (0.0, 0.0, 0.0, 1.0)), None)
check("StopMove sent right after the hung call times out",
      stop_t is not None and stop_t - t0 < 0.2 + 0.1, None if stop_t is None else f"{(stop_t - t0) * 1e3:.0f} ms after STOP")
after = [c for t, c in CALLS if stop_t is not None and t > stop_t]
check("no velocity sent after the STOP", after == [], after)
hung = next((r for r in LOG if r["event"] == "cmd_vel" and not r.get("blocked")), {})
check("hung call is logged as failed with code 3104", hung.get("ok") is False and hung.get("ret") == 3104, hung)
node.destroy_node()

# 3. Velocities queued behind a hung call: only the latest is sent -------------
node = new_node(sdk_timeout_s=0.1)
HANG["n"] = 1
v(node, 0.1, 0.0, 0.0)
time.sleep(0.03)
for x in (0.2, 0.3, 0.4):
    v(node, x, 0.0, 0.0)
drain(node)
got = [c for _, c in CALLS]
check("only the latest velocity is sent after a hang (no stale backlog)",
      got == [("timeout", 0.1, 0.0, 0.0, 1), (0.4, 0.0, 0.0, 1)], got)
node.destroy_node()

w = max(len(n) for n, _, _ in checks)
for name, ok, detail in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {name:{w}s}  {detail if not ok else ''}")
n_ok = sum(ok for _, ok, _ in checks)
print(f"\n{n_ok}/{len(checks)} passed")
sys.exit(0 if n_ok == len(checks) else 1)
