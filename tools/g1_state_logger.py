#!/usr/bin/env python3
"""
Log the G1's onboard state to CSV for a trial. Read-only.

Subscribes to the robot's DDS state topics through unitree_sdk2py. It never
publishes and never sends a command, so it cannot affect the robot. Every row
is stamped with host time (time.time_ns), the clock ROS uses with
use_sim_time:=false, so rows line up with the trial rosbag.

  rt/lowstate (unitree_hg LowState_)   -> lowstate.csv
      tick, IMU (quaternion, gyro, accel, rpy), per-joint q / dq / tau_est,
      and the raw wireless-remote bytes (shows operator input on the remote).
  rt/sportmodestate, rt/odommodestate  -> <topic>.csv
      onboard pose / velocity, only if this firmware publishes them.

Started by run_zed_e2e.sh (window 'trial') when WITH_ROBOT=1. Standalone:

    python3 tools/g1_state_logger.py --out <trial>/robot_state --iface enp4s0

UNVERIFIED on hardware: field names follow the unitree_hg / unitree_go IDL as
used in odom_bridge.py. If a field is missing the row is skipped and the error
is printed once; check lowstate.csv has rows after the first run.
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import threading
import time

N_MOTORS = 29  # g1_29dof


class CsvSink:
    """Buffered CSV writer: DDS callbacks append rows, the main loop flushes."""

    def __init__(self, path: str, header: list[str]):
        self.path = path
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(header)
        self._rows: list[list] = []
        self._lock = threading.Lock()
        self.count = 0
        self.errors = 0
        self.first_error: str | None = None

    def add(self, row: list):
        with self._lock:
            self._rows.append(row)

    def error(self, exc: Exception):
        self.errors += 1
        if self.first_error is None:
            self.first_error = f"{type(exc).__name__}: {exc}"
            print(f"[state_logger] {os.path.basename(self.path)}: row skipped — {self.first_error}",
                  flush=True)

    def flush(self):
        with self._lock:
            rows, self._rows = self._rows, []
        if rows:
            self._w.writerows(rows)
            self._f.flush()
            self.count += len(rows)

    def close(self):
        self.flush()
        self._f.close()


def lowstate_header(n: int) -> list[str]:
    h = ["t_host_ns", "tick"]
    h += [f"imu_quat_{i}" for i in range(4)]
    h += [f"imu_gyro_{a}" for a in "xyz"]
    h += [f"imu_acc_{a}" for a in "xyz"]
    h += [f"imu_rpy_{a}" for a in ("r", "p", "y")]
    h += [f"q_{i}" for i in range(n)]
    h += [f"dq_{i}" for i in range(n)]
    h += [f"tau_{i}" for i in range(n)]
    h += ["remote_hex"]
    return h


SPORT_HEADER = ["t_host_ns", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z",
                "yaw_speed", "rpy_r", "rpy_p", "rpy_y"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--iface", default="", help="host NIC on the robot subnet ('' = auto)")
    ap.add_argument("--motors", type=int, default=N_MOTORS)
    ap.add_argument("--sport-topics", default="rt/sportmodestate,rt/odommodestate")
    args = ap.parse_args()

    try:
        from unitree_sdk2py.core.channel import ChannelFactory, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
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
    low = CsvSink(os.path.join(args.out, "lowstate.csv"), lowstate_header(n))

    def on_low(msg):
        t = time.time_ns()
        try:
            imu = msg.imu_state
            ms = msg.motor_state
            row = [t, msg.tick, *imu.quaternion, *imu.gyroscope, *imu.accelerometer, *imu.rpy]
            row += [ms[i].q for i in range(n)]
            row += [ms[i].dq for i in range(n)]
            row += [ms[i].tau_est for i in range(n)]
            row.append(bytes(msg.wireless_remote).hex())
            low.add(row)
        except Exception as exc:
            low.error(exc)

    subs = []
    s = ChannelSubscriber("rt/lowstate", LowState_)
    s.Init(on_low, 10)
    subs.append(s)

    sinks = [low]
    for topic in [t.strip() for t in args.sport_topics.split(",") if t.strip()]:
        sink = CsvSink(os.path.join(args.out, topic.replace("/", "_") + ".csv"), SPORT_HEADER)

        def on_sport(msg, sink=sink):
            t = time.time_ns()
            try:
                sink.add([t, *msg.position, *msg.velocity, msg.yaw_speed, *msg.imu_state.rpy])
            except Exception as exc:
                sink.error(exc)

        try:
            sub = ChannelSubscriber(topic, SportModeState_)
            sub.Init(on_sport, 10)
            subs.append(sub)
            sinks.append(sink)
        except Exception as exc:
            print(f"[state_logger] cannot subscribe {topic}: {exc}", flush=True)
            sink.close()

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    print(f"[state_logger] logging to {args.out} (read-only, {n} motors)", flush=True)
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
                    print("[state_logger] WARNING: no rt/lowstate rows yet — check NIC / robot link",
                          flush=True)
    finally:
        for k in sinks:
            k.close()
        status = "  ".join(f"{os.path.basename(k.path)}={k.count} rows" for k in sinks)
        print(f"[state_logger] closed: {status}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
