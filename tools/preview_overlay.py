#!/usr/bin/env python3
"""
Offline preview of the demo overlay — no ROS, no robot, no camera.

Synthesises a room, a walking G1, two moving people and a Nav2-style plan that
bends around them, then drives the real `overlay_render.render()` with it. Use
it to iterate on the look, and to smoke-test the renderer after a change.

    python3 tools/preview_overlay.py --out /tmp/preview.mp4 --stills /tmp/stills

The synthetic camera uses the ZED 2i mount from run_zed_e2e.sh: 1.15 m above
the floor, pitched 17.5 deg down.
"""
from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ros2_ws", "src", "g1_detection"))

from g1_detection import overlay_render as R  # noqa: E402

W, H = 1280, 720
INTR = R.Intrinsics(700.0, 700.0, W / 2.0, H / 2.0, W, H)
CAM_HEIGHT = 1.15
CAM_PITCH = 0.3054                      # 17.5 deg down, matches the real mount

R_BASE = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def T_opt_world(cam_xyz, yaw, pitch=CAM_PITCH):
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    cp, sp = np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ R_BASE
    T[:3, 3] = cam_xyz
    return np.linalg.inv(T)


def robot_pose(t):
    """Gentle S-curve down a corridor."""
    x = 0.55 * t
    y = 0.9 * np.sin(0.28 * t)
    yaw = np.arctan2(0.9 * 0.28 * np.cos(0.28 * t), 0.55)
    return x, y, yaw


def human_pose(t, which):
    if which == 0:                       # crosses left-to-right ahead of the robot
        return 6.2 + 0.05 * t, 3.0 - 0.62 * t
    return 9.0 - 0.30 * t, -1.4 + 0.22 * t   # walks toward the robot, drifting in


def project_box(xy, height, half_w, T):
    """Axis-aligned person box -> pixel bbox, or None if off-camera."""
    x, y = xy
    corners = np.array([[x + dx, y + dy, z]
                        for dx in (-half_w, half_w)
                        for dy in (-half_w, half_w)
                        for z in (0.0, height)])
    cam = R.to_camera(corners, T)
    if np.count_nonzero(cam[:, 2] > 0.4) < 8:
        return None
    uv = R.project_camera(cam, INTR)
    x1, y1 = uv[:, 0].min(), uv[:, 1].min()
    x2, y2 = uv[:, 0].max(), uv[:, 1].max()
    if x2 < 0 or x1 > W or y2 < 0 or y1 > H:
        return None
    return [float(x1), float(y1), float(x2), float(y2)]


def draw_room(T):
    """A plain scene so the AR layer has something to sit on."""
    img = np.zeros((H, W, 3), np.uint8)
    img[:, :] = (58, 52, 46)
    cv2.rectangle(img, (0, 0), (W, int(H * 0.42)), (86, 78, 68), -1)
    img = cv2.GaussianBlur(img, (0, 0), 12)

    for gy in np.arange(-6, 7, 1.0):        # floor grid, same projection
        line = np.array([[d, gy] for d in np.arange(-2.0, 26.0, 0.5)])
        cam = R.to_camera(R.ground_polyline(line, 0.0), T)
        for seg in R.clip_polyline_near(cam):
            cv2.polylines(img, [R.project_camera(seg, INTR).astype(np.int32)],
                          False, (74, 68, 60), 1, cv2.LINE_AA)
    for gx in np.arange(-2, 26, 1.0):
        line = np.array([[gx, d] for d in np.arange(-6.0, 6.5, 0.5)])
        cam = R.to_camera(R.ground_polyline(line, 0.0), T)
        for seg in R.clip_polyline_near(cam):
            cv2.polylines(img, [R.project_camera(seg, INTR).astype(np.int32)],
                          False, (74, 68, 60), 1, cv2.LINE_AA)
    return img


def draw_person(img, xy, T):
    """Dark silhouette so the bounding box has a subject."""
    x, y = xy
    body = []
    for z0, z1, hw in ((0.0, 0.9, 0.17), (0.9, 1.45, 0.24), (1.45, 1.75, 0.13)):
        corners = np.array([[x + dx, y + dy, z]
                            for dx in (-hw, hw) for dy in (-hw, hw) for z in (z0, z1)])
        cam = R.to_camera(corners, T)
        if np.count_nonzero(cam[:, 2] > 0.4) < 8:
            continue
        uv = R.project_camera(cam, INTR).astype(np.int32)
        body.append(cv2.convexHull(uv))
    for hull in body:
        cv2.fillConvexPoly(img, hull, (38, 34, 40), cv2.LINE_AA)
        cv2.polylines(img, [hull], True, (22, 20, 24), 2, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/g1_overlay_preview.mp4")
    ap.add_argument("--stills", default="")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--fps", type=float, default=15.0)
    args = ap.parse_args()

    n = int(args.seconds * args.fps)
    # OpenCV moved fourcc onto VideoWriter in 4.10; accept either build.
    fourcc = getattr(cv2.VideoWriter, "fourcc", None) or cv2.VideoWriter_fourcc
    writer = cv2.VideoWriter(args.out, int(fourcc(*"mp4v")), args.fps, (W, H))
    if not writer.isOpened():
        print(f"cannot open {args.out}", file=sys.stderr)
        return 1
    if args.stills:
        os.makedirs(args.stills, exist_ok=True)

    trail: list[tuple[float, float]] = []
    goal = (14.0, 0.0)

    for i in range(n):
        t = i / args.fps
        rx, ry, yaw = robot_pose(t)
        trail.append((rx, ry))
        T = T_opt_world((rx, ry, CAM_HEIGHT), yaw)

        humans_xy = [human_pose(t, 0), human_pose(t, 1)]

        img = draw_room(T)
        for hxy in sorted(humans_xy, key=lambda p: -np.hypot(p[0] - rx, p[1] - ry)):
            draw_person(img, hxy, T)

        # Plan: straight to the goal, pushed aside near whoever is closest.
        ahead = np.linspace(0.0, 1.0, 40)
        plan = []
        for a in ahead:
            px = rx + a * (goal[0] - rx)
            py = ry + a * (goal[1] - ry)
            for hxy in humans_xy:
                d = np.hypot(px - hxy[0], py - hxy[1])
                if d < 1.8:
                    push = (1.8 - d) * 0.9
                    py += push * np.sign(py - hxy[1] or 1.0)
            plan.append([px, py])
        plan = np.array(plan)

        humans = []
        for k, hxy in enumerate(humans_xy):
            bbox = project_box(hxy, 1.75, 0.28, T)
            if bbox is None:
                continue
            nxt = human_pose(t + 1.5, k)
            pred = np.array([[hxy[0] + (nxt[0] - hxy[0]) * s,
                              hxy[1] + (nxt[1] - hxy[1]) * s]
                             for s in np.linspace(0, 1, 10)])
            humans.append(R.Human(
                track_id=k + 1, bbox=bbox,
                distance_m=float(np.hypot(hxy[0] - rx, hxy[1] - ry)),
                confirmed=True, world_xy=hxy, prediction_xy=pred))

        nearest = min((h.distance_m for h in humans), default=None)
        cmd = ("STOP" if nearest is not None and nearest < 1.8 else
               "SLOW_DOWN" if nearest is not None and nearest < 3.8 else
               "NORMAL_OPERATION")

        scene = R.Scene(
            image=img, intrinsics=INTR, T_opt_world=T, ground_z=0.0,
            humans=humans, cmd=cmd, closest_human_m=nearest,
            plan_xy=plan, local_plan_xy=plan[:12],
            travelled_xy=np.array(trail), goal_xy=goal,
            robot_xy=(rx, ry), robot_yaw=yaw,
            speed_mps=0.0 if cmd == "STOP" else (0.22 if cmd == "SLOW_DOWN" else 0.55),
            fps=args.fps, detect_latency_ms=31.0, elapsed_s=t,
            title="UNITREE G1  |  AUTONOMOUS HUMAN AVOIDANCE  |  PREVIEW",
        )
        out = R.render(scene)
        writer.write(out)
        if args.stills and i % max(1, n // 5) == 0:
            cv2.imwrite(os.path.join(args.stills, f"still_{i:04d}.png"), out)

    writer.release()
    print(f"wrote {args.out}  ({n} frames, {W}x{H})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
