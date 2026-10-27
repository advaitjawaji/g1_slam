"""
Demo overlay renderer — egocentric ZED view annotated with everything the stack
knows, composited into a single broadcast-quality frame.

Deliberately free of any ROS import so the geometry and the look can be
developed, unit-tested and previewed off-robot (see `tools/preview_overlay.py`).
`overlay_node.py` is a thin ROS wrapper that fills in `Scene` and calls
`render`.

Layers, back to front:
  1. raw ZED left rectified image
  2. ground-plane AR: robot's Nav2 plan, robot's travelled path, per-human
     predicted trajectory — all projected through the real camera intrinsics
  3. Human_dtp bounding boxes + track tags
  4. bird's-eye inset (the travelled path lives here; it is behind the camera
     and therefore mostly invisible in the egocentric view)
  5. HUD: state banner, metrics, legend

Frame conventions
-----------------
`world` is whatever frame Nav2 plans in (`odom` on this robot). It is Z-up:
ground is the X-Y plane. `optical` is the camera frame camera_info is expressed
in: x right, y down, z forward. Every 3D point enters as world and is pushed
through `T_opt_world` once.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Sequence

import cv2
import numpy as np

# ── Palette (BGR) ─────────────────────────────────────────────────────────
# Chosen to stay separable on a projector and for colour-blind viewers: the
# three trajectory classes differ in hue *and* in lightness.
C_PLAN        = (255, 208, 64)    # cyan      — Nav2 global plan
C_LOCAL       = (200, 255, 160)   # pale mint — controller's local plan
C_TRUE        = (60, 170, 255)    # amber     — path actually walked
C_PRED        = (235, 90, 225)    # magenta   — human predicted trajectory
C_GOAL        = (255, 255, 255)
C_PANEL       = (26, 22, 18)
C_TEXT        = (240, 240, 240)
C_TEXT_DIM    = (170, 170, 170)
C_GRID        = (70, 62, 55)

# Threat-state colours, keyed by /g1/human_cmd.
C_STATE = {
    "NORMAL_OPERATION": (120, 230, 80),
    "SLOW_DOWN":        (0, 190, 255),
    "STOP":             (60, 60, 255),
}
STATE_LABEL = {
    "NORMAL_OPERATION": "CLEAR",
    "SLOW_DOWN":        "SLOWING",
    "STOP":             "STOP",
}

FONT = cv2.FONT_HERSHEY_DUPLEX
Z_NEAR = 0.25          # metres; nothing closer than this is projected
HEADER_H = 62          # title band; labels are kept clear of it

# OpenCV's Hershey fonts are ASCII-only — anything above 0x7F renders as "?".
# Keep every user-visible string in this module (and any `title` passed in)
# inside ASCII; use these instead of the typographic characters.
SEP  = "  |  "
DASH = "--"


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def rescaled_to(self, width: int, height: int) -> "Intrinsics":
        """camera_info describes the full-res stream; the renderer may run on a
        downscaled copy. Scale the pinhole parameters to match."""
        if width == self.width and height == self.height:
            return self
        sx = width / float(self.width)
        sy = height / float(self.height)
        return Intrinsics(self.fx * sx, self.fy * sy,
                          self.cx * sx, self.cy * sy, width, height)


@dataclass
class Human:
    """One Human_dtp track, as published on /g1/detections."""
    track_id: int
    bbox: Sequence[float]                     # x1, y1, x2, y2 in source pixels
    distance_m: float | None = None
    region: str | None = None
    confirmed: bool = False
    world_xy: tuple[float, float] | None = None
    prediction_xy: np.ndarray | None = None   # (N, 2) world ground points


@dataclass
class Scene:
    image: np.ndarray                          # BGR, the frame YOLO actually saw
    intrinsics: Intrinsics | None = None
    T_opt_world: np.ndarray | None = None      # 4x4, world -> camera optical
    ground_z: float = 0.0                      # world Z of the floor

    humans: list[Human] = field(default_factory=list)
    cmd: str = "NORMAL_OPERATION"
    closest_human_m: float | None = None

    plan_xy: np.ndarray | None = None          # (N, 2) Nav2 global plan
    local_plan_xy: np.ndarray | None = None
    travelled_xy: np.ndarray | None = None     # (N, 2) odometry history
    goal_xy: tuple[float, float] | None = None

    robot_xy: tuple[float, float] | None = None
    robot_yaw: float = 0.0
    speed_mps: float = 0.0

    fps: float = 0.0
    detect_latency_ms: float | None = None
    elapsed_s: float = 0.0
    title: str = "UNITREE G1  |  AUTONOMOUS HUMAN AVOIDANCE"
    bev_range_m: float = 6.0
    show_bev: bool = True


# ── Geometry ──────────────────────────────────────────────────────────────

def to_camera(points_world: np.ndarray, T_opt_world: np.ndarray) -> np.ndarray:
    """(N,3) world -> (N,3) camera optical."""
    P = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    if P.size == 0:
        return P.reshape(0, 3)
    Ph = np.hstack([P, np.ones((len(P), 1))])
    return (np.asarray(T_opt_world, dtype=np.float64) @ Ph.T).T[:, :3]


def project_camera(points_cam: np.ndarray, intr: Intrinsics) -> np.ndarray:
    """(N,3) camera optical -> (N,2) pixels. Assumes z > 0 (clip first)."""
    P = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    if P.size == 0:
        return P.reshape(0, 2)
    z = np.maximum(P[:, 2], 1e-6)
    u = intr.fx * P[:, 0] / z + intr.cx
    v = intr.fy * P[:, 1] / z + intr.cy
    return np.stack([u, v], axis=1)


def clip_polyline_near(points_cam: np.ndarray, z_near: float = Z_NEAR) -> list[np.ndarray]:
    """Split a camera-space polyline at the near plane.

    Without this, a vertex behind the camera projects to a mirrored point and
    the path snaps across the screen — the classic AR artifact. Segments that
    straddle z_near are cut at the crossing instead of dropped, so the path
    still runs cleanly off the bottom edge of the frame.
    """
    P = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    if len(P) < 2:
        return [P] if len(P) == 1 and P[0, 2] >= z_near else []

    out: list[np.ndarray] = []
    cur: list[np.ndarray] = []
    for a, b in zip(P[:-1], P[1:]):
        az, bz = a[2], b[2]
        a_in, b_in = az >= z_near, bz >= z_near
        if a_in and b_in:
            if not cur:
                cur.append(a)
            cur.append(b)
        elif a_in and not b_in:
            t = (z_near - az) / (bz - az)
            if not cur:
                cur.append(a)
            cur.append(a + t * (b - a))
            out.append(np.array(cur))
            cur = []
        elif not a_in and b_in:
            t = (z_near - az) / (bz - az)
            cur = [a + t * (b - a), b]
        else:
            if cur:
                out.append(np.array(cur))
                cur = []
    if cur:
        out.append(np.array(cur))
    return [s for s in out if len(s) >= 2]


def ground_polyline(xy: np.ndarray, ground_z: float) -> np.ndarray:
    """(N,2) ground points -> (N,3) world points on the floor plane."""
    P = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    return np.hstack([P, np.full((len(P), 1), ground_z)])


def resample(xy: np.ndarray, step_m: float = 0.12) -> np.ndarray:
    """Densify a sparse path so perspective curvature reads smoothly."""
    P = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(P) < 2:
        return P
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    total = float(seg.sum())
    if total < 1e-6:
        return P
    # Cap at 600: the travelled path grows unbounded over a long demo run and
    # beyond this the extra vertices cost render time without being visible.
    n = int(min(max(total / step_m, len(P)), 600))
    d = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.linspace(0.0, d[-1], n)
    return np.stack([np.interp(t, d, P[:, 0]), np.interp(t, d, P[:, 1])], axis=1)


# ── Drawing primitives ────────────────────────────────────────────────────

def _rounded_mask(shape, x1, y1, x2, y2, radius):
    m = np.zeros(shape[:2], dtype=np.uint8)
    r = int(max(0, min(radius, (x2 - x1) // 2, (y2 - y1) // 2)))
    cv2.rectangle(m, (x1 + r, y1), (x2 - r, y2), 255, -1)
    cv2.rectangle(m, (x1, y1 + r), (x2, y2 - r), 255, -1)
    for cx, cy in ((x1 + r, y1 + r), (x2 - r, y1 + r), (x1 + r, y2 - r), (x2 - r, y2 - r)):
        cv2.circle(m, (cx, cy), r, 255, -1)
    return m


def panel(img, x1, y1, x2, y2, alpha=0.62, color=C_PANEL, radius=12, border=None):
    """Translucent rounded panel, drawn in place."""
    h, w = img.shape[:2]
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(w, int(x2)), min(h, int(y2))
    if x2 <= x1 or y2 <= y1:
        return
    mask = _rounded_mask(img.shape, x1, y1, x2, y2, radius)[y1:y2, x1:x2]
    roi = img[y1:y2, x1:x2]
    tint = np.empty_like(roi)
    tint[:] = color
    blended = cv2.addWeighted(roi, 1.0 - alpha, tint, alpha, 0)
    roi[mask > 0] = blended[mask > 0]
    if border is not None:
        edge = _rounded_mask(img.shape, x1, y1, x2, y2, radius)
        cont, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cont, -1, border, 1, cv2.LINE_AA)


_ASCII_MAP = str.maketrans({
    "·": "|", "•": "*", "—": "--", "–": "-", "→": "->", "°": "deg",
    "“": '"', "”": '"', "‘": "'", "’": "'", "…": "...",
})


def ascii_safe(text: str) -> str:
    """Hershey fonts draw '?' for anything non-ASCII — normalise before drawing."""
    return str(text).translate(_ASCII_MAP).encode("ascii", "replace").decode("ascii")


def tag(img, text, org, fg=C_TEXT, bg=None, scale=0.5, pad=5, thickness=1):
    """Text with an optional filled plate behind it. Returns its box."""
    text = ascii_safe(text)
    (tw, th), base = cv2.getTextSize(text, FONT, scale, thickness)
    x, y = int(org[0]), int(org[1])
    box = (x, y - th - pad, x + tw + 2 * pad, y + base + pad - 2)
    if bg is not None:
        panel(img, box[0], box[1], box[2], box[3], alpha=0.80, color=bg, radius=4)
    cv2.putText(img, text, (x + pad, y - 1), FONT, scale, fg, thickness, cv2.LINE_AA)
    return box


def _depth_thickness(z, base, near=1.5, lo=1, hi=None):
    """Perspective-consistent stroke width: thick underfoot, thin at distance."""
    hi = hi if hi is not None else base * 2
    return int(np.clip(base * (near / max(z, 0.35)), lo, hi))


def draw_ground_path(img, xy, intr, T_opt_world, ground_z, color,
                     base_thickness=5, glow=True, alpha=1.0, dash=0):
    """Project a ground polyline into the image with depth-scaled thickness."""
    if xy is None or len(np.asarray(xy).reshape(-1, 2)) < 2:
        return
    cam = to_camera(ground_polyline(resample(xy), ground_z), T_opt_world)
    layer = img if alpha >= 0.999 else img.copy()

    for seg in clip_polyline_near(cam):
        uv = project_camera(seg, intr)
        pts = uv.astype(np.int32)
        zs = seg[:, 2]
        for i in range(len(pts) - 1):
            if dash and (i // max(dash, 1)) % 2 == 1:
                continue
            p = (int(pts[i][0]), int(pts[i][1]))
            q = (int(pts[i + 1][0]), int(pts[i + 1][1]))
            if not (_on_canvas(p, img.shape) or _on_canvas(q, img.shape)):
                continue
            t = _depth_thickness(0.5 * (zs[i] + zs[i + 1]), base_thickness)
            if glow:
                cv2.line(layer, p, q, (0, 0, 0), t + 4, cv2.LINE_AA)
        for i in range(len(pts) - 1):
            if dash and (i // max(dash, 1)) % 2 == 1:
                continue
            p = (int(pts[i][0]), int(pts[i][1]))
            q = (int(pts[i + 1][0]), int(pts[i + 1][1]))
            if not (_on_canvas(p, img.shape) or _on_canvas(q, img.shape)):
                continue
            t = _depth_thickness(0.5 * (zs[i] + zs[i + 1]), base_thickness)
            cv2.line(layer, p, q, color, t, cv2.LINE_AA)

    if alpha < 0.999:
        cv2.addWeighted(layer, alpha, img, 1.0 - alpha, 0, dst=img)


def _on_canvas(p, shape, margin=4000):
    h, w = shape[:2]
    return -margin <= p[0] <= w + margin and -margin <= p[1] <= h + margin


def draw_ground_marker(img, xy, intr, T_opt_world, ground_z, color,
                       radius_m=0.35, label=None, rings=2):
    """Flat ring(s) on the floor — used for goal and predicted human endpoint."""
    theta = np.linspace(0, 2 * np.pi, 48)
    for k in range(rings):
        r = radius_m * (1.0 + 0.45 * k)
        circle = np.stack([xy[0] + r * np.cos(theta), xy[1] + r * np.sin(theta)], axis=1)
        cam = to_camera(ground_polyline(circle, ground_z), T_opt_world)
        if np.count_nonzero(cam[:, 2] >= Z_NEAR) < 8:
            continue
        for seg in clip_polyline_near(cam):
            uv = project_camera(seg, intr).astype(np.int32)
            cv2.polylines(img, [uv], False, color, max(1, 2 - k), cv2.LINE_AA)
    if label:
        cam = to_camera(np.array([[xy[0], xy[1], ground_z]]), T_opt_world)
        if cam[0, 2] >= Z_NEAR:
            uv = project_camera(cam, intr)[0].astype(int)
            tag(img, label, (uv[0] - 18, uv[1] - 10), fg=color, bg=C_PANEL, scale=0.45)


def human_label(human: Human) -> str:
    label = f"HUMAN {human.track_id}"
    if human.distance_m is not None:
        label += f"  {human.distance_m:.1f}m"
    if not human.confirmed:
        label += "  ?"
    return label


def layout_labels(humans: Sequence[Human], scale=0.48) -> list[int]:
    """Baseline y for each track's tag, nudged down to avoid overlaps.

    Two people standing side by side produce boxes whose tags would sit at the
    same height and overprint each other; walk them left-to-right and push any
    colliding tag onto the next row.
    """
    order = sorted(range(len(humans)), key=lambda i: humans[i].bbox[0])
    placed: list[tuple[float, float, int]] = []          # x1, x2, y
    ys = [0] * len(humans)
    for i in order:
        hm = humans[i]
        x1, y1 = float(hm.bbox[0]), float(hm.bbox[1])
        (tw, th), _ = cv2.getTextSize(ascii_safe(human_label(hm)), FONT, scale, 1)
        row = th + 14
        y = max(y1 - 6, HEADER_H + 22)
        for _ in range(len(humans)):
            hit = any(not (x1 + tw < px1 or x2p < x1) and abs(y - py) < row
                      for px1, x2p, py in placed)
            if not hit:
                break
            y += row
        placed.append((x1, x1 + tw, y))
        ys[i] = int(y)
    return ys


def draw_bbox(img, human: Human, color, label_y: int | None = None):
    """Corner-bracket box + track tag. Reads as instrumentation, not a scribble."""
    x1, y1, x2, y2 = [int(v) for v in human.bbox]
    w, h = x2 - x1, y2 - y1
    if w <= 0 or h <= 0:
        return
    ln = int(np.clip(min(w, h) * 0.28, 10, 46))
    th = 2 if human.confirmed else 1

    # Faint full frame for context; the brackets carry the emphasis.
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), 1, cv2.LINE_AA)
    for (cx, cy, dx, dy) in ((x1, y1, 1, 1), (x2, y1, -1, 1), (x1, y2, 1, -1), (x2, y2, -1, -1)):
        cv2.line(img, (cx, cy), (cx + dx * ln, cy), color, th + 2, cv2.LINE_AA)
        cv2.line(img, (cx, cy), (cx, cy + dy * ln), color, th + 2, cv2.LINE_AA)

    y = label_y if label_y is not None else max(y1 - 6, HEADER_H + 22)
    tag(img, human_label(human), (x1, y),
        fg=(15, 15, 15), bg=color, scale=0.48, thickness=1)


# ── Bird's-eye inset ──────────────────────────────────────────────────────

def draw_bev(img, sc: Scene, size=300, margin=18):
    """Top-down, robot-centric, heading-up mini-map.

    This is where the *travelled* trajectory is legible — in the egocentric
    view it is behind the camera and projects off-frame.
    """
    h, w = img.shape[:2]
    size = int(min(size, w * 0.34, h * 0.52))
    x0, y0 = w - size - margin, h - size - margin
    # Near-opaque: this sits on top of the AR layer and must read as an
    # instrument panel, not let bounding boxes bleed through it.
    panel(img, x0, y0, x0 + size, y0 + size, alpha=0.90, radius=14, border=(90, 80, 70))

    cxp, cyp = x0 + size // 2, y0 + int(size * 0.62)
    rng = max(sc.bev_range_m, 1.0)
    ppm = (size * 0.5) / rng
    rx, ry = sc.robot_xy if sc.robot_xy else (0.0, 0.0)
    ca, sa = np.cos(-sc.robot_yaw + np.pi / 2), np.sin(-sc.robot_yaw + np.pi / 2)

    def W2B(xy):
        P = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        dx, dy = P[:, 0] - rx, P[:, 1] - ry
        ex = dx * ca - dy * sa
        ey = dx * sa + dy * ca
        return np.stack([cxp + ex * ppm, cyp - ey * ppm], axis=1).astype(np.int32)

    clip = (x0 + 2, y0 + 2, x0 + size - 2, y0 + size - 2)
    # Plain Python ints: older OpenCV Python bindings (4.5/4.6, which is what
    # ships on the Humble target) reject numpy integer tuples for clipLine.
    clip_rect = (int(clip[0]), int(clip[1]),
                 int(clip[2] - clip[0]), int(clip[3] - clip[1]))

    def poly(pts, color, thickness):
        if pts is None or len(np.asarray(pts).reshape(-1, 2)) < 2:
            return
        b = W2B(pts)
        for i in range(len(b) - 1):
            p = (int(b[i][0]), int(b[i][1]))
            q = (int(b[i + 1][0]), int(b[i + 1][1]))
            ok, cp, cq = cv2.clipLine(clip_rect, p, q)
            if ok:
                cv2.line(img, cp, cq, color, thickness, cv2.LINE_AA)

    for r in range(1, int(rng) + 1):
        rr = int(r * ppm)
        if rr > 4:
            cv2.circle(img, (cxp, cyp), rr, C_GRID, 1, cv2.LINE_AA)

    poly(sc.travelled_xy, C_TRUE, 2)
    poly(sc.plan_xy, C_PLAN, 2)
    poly(sc.local_plan_xy, C_LOCAL, 1)

    if sc.goal_xy is not None:
        g = W2B([sc.goal_xy])[0]
        if clip[0] < g[0] < clip[2] and clip[1] < g[1] < clip[3]:
            cv2.drawMarker(img, (int(g[0]), int(g[1])), C_GOAL,
                           cv2.MARKER_STAR, 13, 2, cv2.LINE_AA)

    for hm in sc.humans:
        if hm.world_xy is None:
            continue
        poly(hm.prediction_xy, C_PRED, 2)
        p = W2B([hm.world_xy])[0]
        if clip[0] < p[0] < clip[2] and clip[1] < p[1] < clip[3]:
            centre = (int(p[0]), int(p[1]))
            cv2.circle(img, centre, 6, (20, 20, 20), -1, cv2.LINE_AA)
            cv2.circle(img, centre, 5, C_STATE.get(sc.cmd, C_PRED), -1, cv2.LINE_AA)

    nose = np.array([[cxp, cyp - 11], [cxp - 8, cyp + 8], [cxp + 8, cyp + 8]], np.int32)
    cv2.fillConvexPoly(img, nose, (255, 255, 255), cv2.LINE_AA)
    cv2.polylines(img, [nose], True, (30, 30, 30), 1, cv2.LINE_AA)

    tag(img, f"BEV  {rng:.0f}m", (x0 + 10, y0 + 22), fg=C_TEXT_DIM, scale=0.44)


# ── HUD ───────────────────────────────────────────────────────────────────

def draw_hud(img, sc: Scene):
    h, w = img.shape[:2]
    state = sc.cmd if sc.cmd in C_STATE else "NORMAL_OPERATION"
    scol = C_STATE[state]

    panel(img, 0, 0, w, 62, alpha=0.55, radius=0)
    cv2.putText(img, ascii_safe(sc.title), (20, 39), FONT, 0.62, C_TEXT, 1, cv2.LINE_AA)

    # State chip, top-right.
    label = STATE_LABEL[state]
    (tw, _), _ = cv2.getTextSize(label, FONT, 0.72, 2)
    bx2, bx1 = w - 20, w - 20 - tw - 34
    panel(img, bx1, 12, bx2, 50, alpha=0.92, color=scol, radius=8)
    cv2.putText(img, label, (bx1 + 17, 40), FONT, 0.72, (20, 20, 20), 2, cv2.LINE_AA)
    if state == "STOP":
        cv2.rectangle(img, (2, 2), (w - 3, h - 3), scol, 4, cv2.LINE_AA)

    # Metrics strip, bottom-left.
    rows = [
        ("SPEED",   f"{sc.speed_mps:0.2f} m/s"),
        ("NEAREST", DASH if sc.closest_human_m is None else f"{sc.closest_human_m:0.2f} m"),
        ("TRACKS",  f"{len(sc.humans)}"),
        ("PIPE",    f"{sc.fps:0.0f} fps" + ("" if sc.detect_latency_ms is None
                                            else f"{SEP}{sc.detect_latency_ms:0.0f} ms")),
        ("T+",      f"{int(sc.elapsed_s)//60:d}:{int(sc.elapsed_s)%60:02d}"),
    ]
    pw, rh = 232, 26
    px1, py2 = 18, h - 18
    py1 = py2 - rh * len(rows) - 14
    panel(img, px1, py1, px1 + pw, py2, alpha=0.62, radius=12, border=(90, 80, 70))
    for i, (k, v) in enumerate(rows):
        y = py1 + 26 + i * rh
        cv2.putText(img, k, (px1 + 14, y), FONT, 0.42, C_TEXT_DIM, 1, cv2.LINE_AA)
        cv2.putText(img, v, (px1 + 104, y), FONT, 0.5, C_TEXT, 1, cv2.LINE_AA)

    # Legend, directly above the metrics strip.
    items = [("ROBOT PLAN", C_PLAN), ("PATH WALKED", C_TRUE), ("HUMAN FORECAST", C_PRED)]
    ly = py1 - 14
    lw = 196
    panel(img, px1, ly - 22 * len(items) - 10, px1 + lw, ly, alpha=0.62, radius=12,
          border=(90, 80, 70))
    for i, (name, col) in enumerate(items):
        y = ly - 22 * len(items) + 22 * i + 12
        cv2.line(img, (px1 + 14, y - 4), (px1 + 40, y - 4), col, 3, cv2.LINE_AA)
        cv2.putText(img, name, (px1 + 50, y), FONT, 0.42, C_TEXT, 1, cv2.LINE_AA)


# ── Top level ─────────────────────────────────────────────────────────────

def render(sc: Scene) -> np.ndarray:
    """Compose the full annotated frame. Never raises on missing inputs."""
    img = np.ascontiguousarray(sc.image.copy())
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    h, w = img.shape[:2]
    state_col = C_STATE.get(sc.cmd, C_STATE["NORMAL_OPERATION"])

    can_project = sc.intrinsics is not None and sc.T_opt_world is not None
    if can_project:
        intr = sc.intrinsics.rescaled_to(w, h)
        T = sc.T_opt_world
        gz = sc.ground_z

        # Travelled path first — it sits under everything and is usually behind
        # the camera; it only appears when the robot turns back on itself.
        draw_ground_path(img, sc.travelled_xy, intr, T, gz, C_TRUE,
                         base_thickness=4, alpha=0.85)
        draw_ground_path(img, sc.plan_xy, intr, T, gz, C_PLAN, base_thickness=6)
        draw_ground_path(img, sc.local_plan_xy, intr, T, gz, C_LOCAL,
                         base_thickness=3, glow=False, alpha=0.9)

        if sc.goal_xy is not None:
            draw_ground_marker(img, sc.goal_xy, intr, T, gz, C_GOAL,
                               radius_m=0.30, label="GOAL")

        for hm in sc.humans:
            if hm.prediction_xy is None or len(np.asarray(hm.prediction_xy).reshape(-1, 2)) < 2:
                continue
            draw_ground_path(img, hm.prediction_xy, intr, T, gz, C_PRED,
                             base_thickness=4, dash=3)
            end = np.asarray(hm.prediction_xy).reshape(-1, 2)[-1]
            draw_ground_marker(img, end, intr, T, gz, C_PRED,
                               radius_m=0.28, label="1.5s", rings=1)

    label_ys = layout_labels(sc.humans)
    for hm, ly in zip(sc.humans, label_ys):
        draw_bbox(img, hm, state_col, label_y=ly)

    if sc.show_bev:
        draw_bev(img, sc)
    draw_hud(img, sc)
    return img
