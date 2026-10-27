#!/usr/bin/env python3
"""
Checks the G1 state logger's row builders against messages shaped like the
unitree_hg IDL (LowState_ / MotorState_ / IMUState_), without the SDK or a robot.

    python3 tools/test_state_logger_rows.py
"""
import csv
import dataclasses
import os
import sys
import tempfile
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1_state_logger as L  # noqa: E402

checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))


# Field names exactly as in unitree_sdk2py/idl/unitree_hg/msg/dds_ (master).
def motor(i):
    return NS(mode=1, q=0.1 * i, dq=0.01 * i, ddq=0.001 * i, tau_est=0.5 * i,
              temperature=[30 + i, 31 + i], vol=48.0, sensor=[i, 2 * i],
              motorstate=0, reserve=[0] * 4)


def imu():
    return NS(quaternion=[1.0, 0.0, 0.0, 0.0], gyroscope=[0.1, 0.2, 0.3],
              accelerometer=[0.0, 0.0, 9.81], rpy=[0.01, 0.02, 0.03], temperature=40)


def lowstate(tick=7, flag_3=0, q_shift=0.0):
    ms = [motor(i) for i in range(35)]
    ms[3].motorstate = flag_3
    for m in ms:
        m.q += q_shift
    return NS(version=[0, 0], mode_pr=0, mode_machine=5, tick=tick, imu_state=imu(),
              motor_state=ms, wireless_remote=[0] * 40, reserve=[0] * 4, crc=0)


n = L.N_MOTORS
msg = lowstate()

hdr = L.lowstate_header(n)
row = L.lowstate_row(123, msg, n)
check("lowstate row matches its header", len(row) == len(hdr), f"{len(row)} vs {len(hdr)}")
check("all 35 motor slots are logged", "q_34" in hdr and "ddq_34" in hdr and "tau_34" in hdr)
r = dict(zip(hdr, row))
check("joint values land in the right columns",
      abs(r["q_5"] - 0.5) < 1e-9 and abs(r["dq_5"] - 0.05) < 1e-9
      and abs(r["ddq_2"] - 0.002) < 1e-9 and r["tau_10"] == 5.0)
check("IMU and modes land in the right columns",
      r["imu_acc_z"] == 9.81 and r["imu_rpy_y"] == 0.03 and r["mode_machine"] == 5 and r["tick"] == 7)
check("remote bytes are hex-encoded", r["remote_hex"] == "00" * 40)

hh = L.health_header(n)
hrow = L.health_row(1, msg, n, "first")
check("motor_health row matches its header", len(hrow) == len(hh), f"{len(hrow)} vs {len(hh)}")
h = dict(zip(hh, hrow))
check("temperatures, voltage and flags land in the right columns",
      h["temp0_4"] == 34 and h["temp1_4"] == 35 and h["vol_0"] == 48.0
      and h["sensor1_3"] == 6 and h["motorstate_3"] == 0 and h["imu_temp"] == 40)
check("an error-flag change triggers an immediate health row",
      L.health_key(lowstate(flag_3=0), n) != L.health_key(lowstate(flag_3=8), n))
check("a joint-angle change alone does not",
      L.health_key(lowstate(), n) == L.health_key(lowstate(q_shift=0.3), n))

check("torso-IMU row matches its header", len(L.imu_row(5, imu())) == len(L.IMU_HEADER))
sport = NS(position=[1.0, 2.0, 0.0], velocity=[0.3, 0.0, 0.0], yaw_speed=0.1, imu_state=imu())
check("sport-state row matches its header", len(L.sport_row(5, sport)) == len(L.SPORT_HEADER))


# --extra topics are flattened generically; IDL messages are dataclasses.
@dataclasses.dataclass
class Cell:
    v: float
    t: int


@dataclasses.dataclass
class Bms:
    status: int
    cells: list
    raw: bytes


pairs = dict(L.flatten(Bms(1, [Cell(3.7, 25), Cell(3.6, 26)], b"\x01\x02")))
check("flatten names every nested/array field",
      pairs == {"status": 1, "cells.0.v": 3.7, "cells.0.t": 25,
                "cells.1.v": 3.6, "cells.1.t": 26, "raw": "0102"}, str(pairs))

with tempfile.TemporaryDirectory() as d:
    p = os.path.join(d, "x.csv")
    sink = L.CsvSink(p)                      # header comes from the first message
    sink.set_header(["t_host_ns", "a"])
    sink.add([1, 2.5])
    sink.add([2, 3.5])
    sink.close()
    with open(p, newline="") as f:
        rows = list(csv.reader(f))
    check("CsvSink writes a late header then the rows",
          rows == [["t_host_ns", "a"], ["1", "2.5"], ["2", "3.5"]], str(rows))

width = max(len(nm) for nm, _, _ in checks)
failed = 0
for nm, ok, detail in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {nm.ljust(width)}  {detail}")
    failed += (not ok)
print(f"\n{len(checks) - failed}/{len(checks)} passed")
sys.exit(1 if failed else 0)
