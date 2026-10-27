#!/usr/bin/env python3
"""
Sample the computer's load during a trial into a CSV.

Per sample: whole-machine CPU (mean and busiest core), load average, memory,
CPU temperature; GPU utilisation, memory, temperature, power and SM clock (via
nvidia-smi); and CPU % / resident memory for each pipeline component, so a
latency change can be attributed to (or ruled out from) recording overhead.

Stamped with host time (t_host_ns), the same clock as the trial rosbag.
Started by run_zed_e2e.sh (window 'trial'). Standalone:

    python3 tools/sys_monitor.py --out <trial>/system.csv --hz 2
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

try:
    import psutil
except ImportError:
    sys.exit("psutil is required (sudo apt install python3-psutil)")

# Component -> command-line substrings. Checked in order, first match wins, so
# processes whose arguments mention /zed/... topics are claimed before 'zed'.
GROUPS = [
    ("bag_record",     ["bag record"]),
    ("odom_relay",     ["topic_tools"]),
    ("detection",      ["detection_node"]),
    ("overlay",        ["overlay_node"]),
    ("human_obstacle", ["human_obstacle_node"]),
    ("robot_node",     ["robot_node"]),
    ("state_logger",   ["g1_state_logger"]),
    ("sys_monitor",    ["sys_monitor"]),
    ("nav2",           ["controller_server", "planner_server", "bt_navigator",
                        "behavior_server", "smoother_server", "velocity_smoother",
                        "waypoint_follower", "lifecycle_manager",
                        "collision_monitor", "opennav_docking", "route_server"]),
    ("zed",            ["zed"]),
    ("rviz",           ["rviz2"]),
    ("tf_rsp",         ["static_transform_publisher", "robot_state_publisher"]),
]
GPU_FIELDS = ["utilization.gpu", "utilization.memory", "memory.used",
              "temperature.gpu", "power.draw", "clocks.sm"]
GPU_COLS = ["gpu_util_pct", "gpu_mem_util_pct", "gpu_mem_used_mb",
            "gpu_temp_c", "gpu_power_w", "gpu_sm_clock_mhz"]


def classify(cmdline: str) -> str | None:
    if "cat >>" in cmdline:          # tmux pane-log mirrors (logs/<name>.log), not components
        return None
    for name, pats in GROUPS:
        if any(p in cmdline for p in pats):
            return name
    return None


def gpu_sample() -> list:
    if not shutil.which("nvidia-smi"):
        return [""] * len(GPU_COLS)
    try:
        out = subprocess.run(
            ["nvidia-smi", f"--query-gpu={','.join(GPU_FIELDS)}",
             "--format=csv,noheader,nounits", "-i", "0"],
            capture_output=True, text=True, timeout=2.0).stdout.strip().splitlines()
        vals = [v.strip() for v in out[0].split(",")] if out else []
    except (subprocess.SubprocessError, OSError, IndexError):
        vals = []
    clean = []
    for v in vals[:len(GPU_COLS)]:
        try:
            clean.append(float(v))
        except ValueError:          # "[N/A]" on some boards
            clean.append("")
    return clean + [""] * (len(GPU_COLS) - len(clean))


def cpu_temp() -> float | str:
    try:
        temps = psutil.sensors_temperatures()
    except (AttributeError, OSError):
        temps = {}
    vals = [t.current for name in ("coretemp", "k10temp", "cpu_thermal", "acpitz")
            for t in temps.get(name, []) if t.current is not None]
    if vals:
        return max(vals)
    # Jetson: the CPU sensor is a thermal zone, which psutil skips when any
    # hwmon sensor exists.
    for zone in glob.glob("/sys/class/thermal/thermal_zone*"):
        try:
            with open(os.path.join(zone, "type")) as f:
                if f.read().strip() != "cpu-thermal":
                    continue
            with open(os.path.join(zone, "temp")) as f:
                return int(f.read()) / 1000.0
        except (OSError, ValueError):
            continue
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output CSV path")
    ap.add_argument("--hz", type=float, default=2.0)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    names = [g for g, _ in GROUPS]
    header = (["t_host_ns", "cpu_pct", "cpu_max_core_pct", "load1", "mem_used_pct",
               "cpu_temp_c"] + GPU_COLS
              + [f"{g}_cpu_pct" for g in names] + [f"{g}_rss_mb" for g in names])

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    procs: dict[int, tuple[psutil.Process, str]] = {}
    psutil.cpu_percent(percpu=True)          # prime the system-wide counters
    period = 1.0 / max(args.hz, 0.1)
    rows = 0
    print(f"[sys_monitor] sampling at {args.hz:g} Hz -> {args.out}", flush=True)

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        while not stop.is_set():
            t_next = time.time() + period

            # Track pipeline processes; new ones are primed and counted next sample.
            live = set()
            for p in psutil.process_iter(["pid", "cmdline"]):
                try:
                    cmd = " ".join(p.info["cmdline"] or [])
                except (psutil.Error, TypeError):
                    continue
                group = classify(cmd)
                if group is None:
                    continue
                live.add(p.pid)
                if p.pid not in procs:
                    procs[p.pid] = (p, group)
                    try:
                        p.cpu_percent(None)
                    except psutil.Error:
                        pass
            cpu_by = dict.fromkeys(names, 0.0)
            rss_by = dict.fromkeys(names, 0.0)
            for pid in list(procs):
                if pid not in live:
                    del procs[pid]
                    continue
                p, group = procs[pid]
                try:
                    cpu_by[group] += p.cpu_percent(None)
                    rss_by[group] += p.memory_info().rss / 1e6
                except psutil.Error:
                    del procs[pid]

            per_core = psutil.cpu_percent(percpu=True)
            row = [time.time_ns(),
                   round(sum(per_core) / max(len(per_core), 1), 1),
                   max(per_core) if per_core else "",
                   round(os.getloadavg()[0], 2) if hasattr(os, "getloadavg") else "",
                   psutil.virtual_memory().percent,
                   cpu_temp()]
            row += gpu_sample()
            row += [round(cpu_by[g], 1) for g in names]
            row += [round(rss_by[g], 1) for g in names]
            w.writerow(row)
            f.flush()
            rows += 1
            stop.wait(max(0.0, t_next - time.time()))

    print(f"[sys_monitor] closed: {rows} samples", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
