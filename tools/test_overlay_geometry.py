#!/usr/bin/env python3
"""
Ground-truth checks for the demo overlay's projection maths.

Runs without ROS — `python3 tools/test_overlay_geometry.py`. Values are derived
by hand from the pinhole model rather than from the renderer, so this catches a
sign flip or an axis swap that a visual preview would happily hide.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "g1_detection"))

from g1_detection import overlay_render as R  # noqa: E402


FX = FY = 500.0
CX, CY = 640.0, 360.0
INTR = R.Intrinsics(FX, FY, CX, CY, 1280, 720)

# Optical axes expressed in a Z-up, X-forward world: optical +z is world +x,
# optical +x is world -y (right), optical +y is world -z (down).
R_BASE = np.array([[0.0, 0.0, 1.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0]])


def T_opt_world(cam_xyz, yaw=0.0, pitch=0.0):
    """Inverse of the camera pose — what the renderer consumes."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    cp, sp = np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rwo = Rz @ Ry @ R_BASE
    T = np.eye(4)
    T[:3, :3] = Rwo
    T[:3, 3] = cam_xyz
    return np.linalg.inv(T)


checks = []


def check(name, cond, detail=""):
    checks.append((name, bool(cond), detail))


# ── 1. A point straight ahead on the floor ────────────────────────────────
# Camera 1.25 m up, level, looking along +x. A floor point 2 m ahead is 1.25 m
# *below* the optical centre, so it must land below the principal point by
# fy * 1.25 / 2 = 312.5 px, dead centre horizontally.
T = T_opt_world((0.0, 0.0, 1.25))
cam = R.to_camera(np.array([[2.0, 0.0, 0.0]]), T)
uv = R.project_camera(cam, INTR)[0]
check("floor point ahead: depth", np.isclose(cam[0, 2], 2.0), f"z={cam[0,2]:.4f}")
check("floor point ahead: u == cx", np.isclose(uv[0], CX), f"u={uv[0]:.2f}")
check("floor point ahead: v == cy + fy*h/z",
      np.isclose(uv[1], CY + FY * 1.25 / 2.0), f"v={uv[1]:.2f} want={CY + FY*1.25/2:.2f}")

# ── 2. Left/right handedness ──────────────────────────────────────────────
# World +y is to the robot's left, which must appear left of centre (u < cx).
uv_left = R.project_camera(R.to_camera(np.array([[2.0, 0.5, 0.0]]), T), INTR)[0]
uv_right = R.project_camera(R.to_camera(np.array([[2.0, -0.5, 0.0]]), T), INTR)[0]
check("world +y renders left of centre", uv_left[0] < CX, f"u={uv_left[0]:.2f}")
check("world -y renders right of centre", uv_right[0] > CX, f"u={uv_right[0]:.2f}")
check("left/right symmetric about cx",
      np.isclose(CX - uv_left[0], uv_right[0] - CX), f"{uv_left[0]:.2f} {uv_right[0]:.2f}")

# ── 3. Farther away => closer to the horizon ──────────────────────────────
v_near = R.project_camera(R.to_camera(np.array([[1.5, 0.0, 0.0]]), T), INTR)[0][1]
v_far = R.project_camera(R.to_camera(np.array([[8.0, 0.0, 0.0]]), T), INTR)[0][1]
check("floor recedes toward horizon", v_far < v_near and v_far > CY,
      f"near={v_near:.1f} far={v_far:.1f} horizon={CY:.1f}")

# ── 4. Yaw ────────────────────────────────────────────────────────────────
# Turn the robot 90 deg left; a point that was ahead is now to its right.
Ty = T_opt_world((0.0, 0.0, 1.25), yaw=np.pi / 2)
cam_y = R.to_camera(np.array([[2.0, 0.0, 0.0]]), Ty)
check("yaw 90deg puts the old forward point on the right",
      cam_y[0, 2] < 0.1 and cam_y[0, 0] > 1.0, f"cam={cam_y[0]}")

# ── 5. Pitch ──────────────────────────────────────────────────────────────
# Pitching the camera down must push the floor point back toward the centre.
Tp = T_opt_world((0.0, 0.0, 1.25), pitch=0.3054)   # the 17.5 deg ZED mount
v_pitched = R.project_camera(R.to_camera(np.array([[2.0, 0.0, 0.0]]), Tp), INTR)[0][1]
check("pitching down raises the floor point in-frame",
      v_pitched < uv[1], f"level={uv[1]:.1f} pitched={v_pitched:.1f}")

# ── 6. Near-plane clipping ────────────────────────────────────────────────
poly = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, 3.0]])
segs = R.clip_polyline_near(poly)
check("straddling segment is clipped, not dropped", len(segs) == 1, f"{len(segs)} segs")
if segs:
    check("clip lands exactly on the near plane",
          np.isclose(segs[0][0, 2], R.Z_NEAR), f"z={segs[0][0,2]}")

check("fully-behind segment is dropped",
      len(R.clip_polyline_near(np.array([[0.0, 0.0, -2.0], [0.0, 0.0, -1.0]]))) == 0)
check("fully-ahead segment is kept whole",
      len(R.clip_polyline_near(np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 5.0]]))) == 1)

# A path that starts behind, comes forward, and goes behind again -> 1 run.
zig = np.array([[0, 0, -1.0], [0, 0, 2.0], [0, 0, 4.0], [0, 0, -3.0]], dtype=float)
check("re-entering path yields one contiguous run", len(R.clip_polyline_near(zig)) == 1,
      f"{len(R.clip_polyline_near(zig))}")

# ── 7. Resampling ─────────────────────────────────────────────────────────
line = np.array([[0.0, 0.0], [3.0, 0.0]])
rs = R.resample(line, step_m=0.1)
check("resample keeps endpoints",
      np.allclose(rs[0], line[0]) and np.allclose(rs[-1], line[-1]))
check("resample densifies", len(rs) > 20, f"n={len(rs)}")
check("resample tolerates a degenerate path", len(R.resample(np.array([[1.0, 1.0]]))) == 1)

# ── 8. Intrinsics rescaling ───────────────────────────────────────────────
half = INTR.rescaled_to(640, 360)
check("intrinsics scale with resolution",
      np.isclose(half.fx, FX / 2) and np.isclose(half.cx, CX / 2))
check("rescale to same size is identity", INTR.rescaled_to(1280, 720) is INTR)

# ── 9. A ground path really does land on the floor pixels ─────────────────
# Independent re-derivation: for a level camera at height h, a floor point at
# distance d projects to v = cy + fy*h/d. Walk a straight path and confirm.
path = np.array([[d, 0.0] for d in (1.0, 2.0, 4.0, 8.0)])
cam_path = R.to_camera(R.ground_polyline(path, 0.0), T)
uv_path = R.project_camera(cam_path, INTR)
want = [CY + FY * 1.25 / d for d in (1.0, 2.0, 4.0, 8.0)]
check("ground_polyline + projection matches the closed form",
      np.allclose(uv_path[:, 1], want), f"{uv_path[:,1]} vs {want}")

# ── Report ────────────────────────────────────────────────────────────────
width = max(len(n) for n, _, _ in checks)
failed = 0
for name, ok, detail in checks:
    print(f"  {'PASS' if ok else 'FAIL'}  {name.ljust(width)}  {detail}")
    failed += (not ok)

print(f"\n{len(checks) - failed}/{len(checks)} passed")
sys.exit(1 if failed else 0)
