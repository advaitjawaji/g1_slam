#!/usr/bin/env python3
"""
Log the G1's onboard state to CSV for a trial. Read-only.

Subscribes to the robot's DDS state topics through unitree_sdk2py. It never
publishes and never sends a command, so it cannot affect the robot. Every row
is stamped with host time (t_host_ns = time.time_ns), the clock ROS uses with
use_sim_time:=false, so rows line up with the trial rosbag; `tick` is the
robot's own counter.

  rt/lowstate (LowState_)            -> lowstate.csv       every message
      tick, robot modes, pelvis IMU (quaternion, gyro, accel, rpy), and for all
      35 motor slots q, dq, ddq, tau_est, plus the raw wireless-remote bytes.
                                     -> motor_health.csv   at --health-hz, and at
      once whenever the robot mode or any motor's mode / error flags change:
      IMU temperature and, per motor, mode, temperature[2], voltage, sensor[2],
      motorstate (error flags).
  rt/secondary_imu (IMUState_)       -> secondary_imu.csv  torso IMU
  rt/sportmodestate, rt/odommodestate -> <topic>.csv       onboard pose/velocity,
      only if this firmware publishes them.
  --extra TOPIC=PKG.TYPE             -> <topic>.csv        any other state topic,
      every field, e.g.  --extra rt/lf/bmsstate=unitree_hg.BmsState_  once the
      battery topic name is confirmed on this robot.

Started by run_zed_e2e.sh (window 'trial') when WITH_ROBOT=1. Standalone:

    python3 tools/g1_state_logger.py --out <trial>/robot_state --iface enp4s0

Field names follow the unitree_hg IDL in unitree_sdk2_python; the topic names
come from Unitree's G1 SDK examples. UNVERIFIED on hardware: check that every
CSV has rows after the first run (tools/trial_check.py reports it).
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import importlib
import os
import signal
import sys
import threading
import time

N_MOTORS = 35  # LowState_.motor_state slots; the 29-DoF G1 uses the first 29
HEALTH_FIELDS = ("mode", "temp0", "temp1", "vol", "sensor0", "sensor1", "motorstate")


# ── Row builders (pure, so tools/test_state_logger_rows.py can check them) ──

def lowstate_header(n: int) -> list[str]:
    h = ["t_host_ns", "tick", "mode_pr", "mode_machine"]
    h += [f"imu_quat_{i}" for i in range(4)]
    h += [f"imu_gyro_{a}" for a in "xyz"]
    h += [f"imu_acc_{a}" for a in "xyz"]
    h += [f"imu_rpy_{a}" for a in ("r", "p", "y")]
    for key in ("q", "dq", "ddq", "tau"):
        h += [f"{key}_{i}" for i in range(n)]
    h.append("remote_hex")
    return h


def lowstate_row(t: int, msg, n: int) -> list:
    imu, ms = msg.imu_state, msg.motor_state
    row = [t, msg.tick, msg.mode_pr, msg.mode_machine,
           *imu.quaternion, *imu.gyroscope, *imu.accelerometer, *imu.rpy]
    row += [ms[i].q for i in range(n)]
    row += [ms[i].dq for i in range(n)]
    row += [ms[i].ddq for i in range(n)]
    row += [ms[i].tau_est for i in range(n)]
    row.append(bytes(msg.wireless_remote).hex())
    return row


def health_header(n: int) -> list[str]:
    h = ["t_host_ns", "tick", "reason", "mode_pr", "mode_machine", "imu_temp"]
    for f in HEALTH_FIELDS:
        h += [f"{f}_{i}" for i in range(n)]
    return h


def health_key(msg, n: int) -> tuple:
    """What a change in should be logged immediately: modes and error flags."""
    ms = msg.motor_state
    return (msg.mode_pr, msg.mode_machine,
            tuple(ms[i].mode for i in range(n)),
            tuple(ms[i].motorstate for i in range(n)))


def health_row(t: int, msg, n: int, reason: str) -> list:
    ms = msg.motor_state
    row = [t, msg.tick, reason, msg.mode_pr, msg.mode_machine, msg.imu_state.temperature]
    row += [ms[i].mode for i in range(n)]
    row += [ms[i].temperature[0] for i in range(n)]
    row += [ms[i].temperature[1] for i in range(n)]
    row += [ms[i].vol for i in range(n)]
    row += [ms[i].sensor[0] for i in range(n)]
    row += [ms[i].sensor[1] for i in range(n)]
    row += [ms[i].motorstate for i in range(n)]
    return row


IMU_HEADER = (["t_host_ns"] + [f"quat_{i}" for i in range(4)]
              + [f"gyro_{a}" for a in "xyz"] + [f"acc_{a}" for a in "xyz"]
              + [f"rpy_{a}" for a in ("r", "p", "y")] + ["temp"])


def imu_row(t: int, imu) -> list:
    return [t, *imu.quaternion, *imu.gyroscope, *imu.accelerometer, *imu.rpy, imu.temperature]


SPORT_HEADER = ["t_host_ns", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z",
                "yaw_speed", "rpy_r", "rpy_p", "rpy_y"]


def sport_row(t: int, msg) -> list:
    return [t, *msg.position, *msg.velocity, msg.yaw_speed, *msg.imu_state.rpy]


def flatten(obj, prefix: str = ""):
    """(name, value) for every leaf field of an IDL message; arrays are indexed."""
    if dataclasses.is_dataclass(obj):
        for f in dataclasses.fields(obj):
            yield from flatten(getattr(obj, f.name), f"{prefix}{f.name}.")
    elif isinstance(obj, (bytes, bytearray)):
        yield prefix[:-1], obj.hex()
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            yield from flatten(v, f"{prefix}{i}.")
    else:
        yield prefix[:-1], obj


# ── Output ────────────────────────────────────────────────────────────────

class CsvSink:
    """Buffered CSV writer: DDS callbacks append rows, the main loop flushes.

    The header may be given up front or, for --extra topics, taken from the
    first message (set_header)."""

    def __init__(self, path: str, header: list[str] | None = None):
        self.path = path
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._header = header
        self._header_written = False
        self._rows: list[list] = []
        self._lock = threading.Lock()
        self.count = 0
        self.errors = 0
        self.first_error: str | None = None

    @property
    def has_header(self) -> bool:
        return self._header is not None

    def set_header(self, header: list[str]):
        with self._lock:
            if self._header is None:
                self._header = header

    def add(self, row: list):
        with self._lock:
            self._rows.append(row)

    def error(self, exc: Exception):
        self.errors += 1
        if self.first_error is None:
            self.first_error = f"{type(exc).__name__}: {exc}"
            print(f"[state_logger] {os.path.basename(self.path)}: row skipped - "
                  f"{self.first_error}", flush=True)

    def flush(self):
        with self._lock:
            rows, self._rows = self._rows, []
            header = self._header
        if header is not None and not self._header_written:
            self._w.writerow(header)
            self._header_written = True
        if rows:
            self._w.writerows(rows)
            self.count += len(rows)
        self._f.flush()

    def close(self):
        self.flush()
        self._f.close()


def topic_file(out: str, topic: str) -> str:
    return os.path.join(out, topic.replace("/", "_") + ".csv")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--iface", default="", help="host NIC on the robot subnet ('' = auto)")
    ap.add_argument("--motors", type=int, default=N_MOTORS)
    ap.add_argument("--health-hz", type=float, default=10.0,
                    help="periodic rate of motor_health.csv (changes are logged at once)")
    ap.add_argument("--secondary-imu", default="rt/secondary_imu",
                    help="torso IMU topic ('' to skip)")
    ap.add_argument("--sport-topics", default="rt/sportmodestate,rt/odommodestate")
    ap.add_argument("--extra", action="append", default=[], metavar="TOPIC=PKG.TYPE",
                    help="log another state topic, every field (repeatable)")
    args = ap.parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, LowState_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    except Exception as exc:
        print(f"[state_logger] unitree_sdk2py not importable ({exc}); nothing logged.", flush=True)
        return 1

    os.makedirs(args.out, exist_ok=True)
    if args.iface:
        ChannelFactory().Init(0, args.iface)
    else:
        ChannelFactory().Init(0)

    n = args.motors
    subs, sinks = [], []

    def subscribe(topic, msg_type, handler):
        try:
            sub = ChannelSubscriber(topic, msg_type)
            sub.Init(handler, 10)
            subs.append(sub)
            return True
        except Exception as exc:
            print(f"[state_logger] cannot subscribe {topic}: {exc}", flush=True)
            return False

    # rt/lowstate -> lowstate.csv (every message) + motor_health.csv
    low = CsvSink(os.path.join(args.out, "lowstate.csv"), lowstate_header(n))
    health = CsvSink(os.path.join(args.out, "motor_health.csv"), health_header(n))
    sinks += [low, health]
    period_ns = int(1e9 / max(args.health_hz, 0.1))
    hstate = {"key": None, "t": 0}

    def on_low(msg):
        t = time.time_ns()
        try:
            low.add(lowstate_row(t, msg, n))
        except Exception as exc:
            low.error(exc)
        try:
            key = health_key(msg, n)
            if key != hstate["key"]:
                health.add(health_row(t, msg, n, "first" if hstate["key"] is None else "change"))
                hstate["key"], hstate["t"] = key, t
            elif t - hstate["t"] >= period_ns:
                health.add(health_row(t, msg, n, "periodic"))
                hstate["t"] = t
        except Exception as exc:
            health.error(exc)

    subscribe("rt/lowstate", LowState_, on_low)

    # Torso IMU
    if args.secondary_imu:
        imu_sink = CsvSink(os.path.join(args.out, "secondary_imu.csv"), IMU_HEADER)

        def on_imu(msg, sink=imu_sink):
            t = time.time_ns()
            try:
                sink.add(imu_row(t, msg))
            except Exception as exc:
                sink.error(exc)

        if subscribe(args.secondary_imu, IMUState_, on_imu):
            sinks.append(imu_sink)
        else:
            imu_sink.close()

    # Onboard pose / velocity, if the firmware publishes it
    for topic in [t.strip() for t in args.sport_topics.split(",") if t.strip()]:
        sink = CsvSink(topic_file(args.out, topic), SPORT_HEADER)

        def on_sport(msg, sink=sink):
            t = time.time_ns()
            try:
                sink.add(sport_row(t, msg))
            except Exception as exc:
                sink.error(exc)

        if subscribe(topic, SportModeState_, on_sport):
            sinks.append(sink)
        else:
            sink.close()

    # Any other state topic, every field
    for spec in args.extra:
        try:
            topic, type_path = spec.split("=", 1)
            pkg, type_name = type_path.rsplit(".", 1)
            msg_type = getattr(importlib.import_module(f"unitree_sdk2py.idl.{pkg}.msg.dds_"),
                               type_name)
        except Exception as exc:
            print(f"[state_logger] bad --extra '{spec}': {exc}", flush=True)
            continue
        sink = CsvSink(topic_file(args.out, topic))

        def on_extra(msg, sink=sink):
            t = time.time_ns()
            try:
                pairs = list(flatten(msg))
                if not sink.has_header:
                    sink.set_header(["t_host_ns"] + [k for k, _ in pairs])
                sink.add([t] + [v for _, v in pairs])
            except Exception as exc:
                sink.error(exc)

        if subscribe(topic, msg_type, on_extra):
            sinks.append(sink)
        else:
            sink.close()

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    print(f"[state_logger] logging to {args.out} (read-only, {n} motor slots)", flush=True)
    t0 = time.time()
    last_report = t0
    try:
        while not stop.is_set():
            time.sleep(0.5)
            for k in sinks:
                k.flush()
            now = time.time()
            if now - last_report >= 5.0:
                last_report = now
                status = "  ".join(f"{os.path.basename(k.path)}={k.count}" for k in sinks)
                print(f"[state_logger] {now - t0:6.0f}s  {status}", flush=True)
                if low.count == 0 and now - t0 > 5.0:
                    print("[state_logger] WARNING: no rt/lowstate rows yet - check NIC / robot link",
                          flush=True)
    finally:
        for k in sinks:
            k.close()
        status = "  ".join(f"{os.path.basename(k.path)}={k.count} rows" for k in sinks)
        print(f"[state_logger] closed: {status}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
