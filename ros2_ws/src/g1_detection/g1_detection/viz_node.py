from __future__ import annotations

"""
viz_node — composite demo visualizer for the G1 human-avoidance stack.

Renders ONE polished frame combining:
  • Egocentric ZED view with per-human bounding boxes (color-coded by track ID),
    ID + live distance labels, an image-space motion arrow, and a danger tint.
  • A robot-centric bird's-eye view (BEV) with the robot, detected humans, their
    PREDICTED trajectories (fading trails), the Nav2 plan, the traveled path,
    and danger rings.
  • A big STOP / SLOW_DOWN / NORMAL_OPERATION status banner + a stats strip.

Output: published as sensor_msgs/Image on /viz/overlay AND (optionally) written
to an mp4 for the demo video.

Reuses g1_detection.detection.HumanXZPredictor for perception (same model as
detection_node), so boxes/IDs/predictions match the real avoidance pipeline.

Frame notes: in the USE_ZED_ODOM=1 / no-map wiring, the predictor has no
map<-camera TF, so its position_xz_m / predictions_xz are CAMERA-relative
(x = right, z = forward) — exactly robot-centric, which is what the BEV wants.
The Nav2 /plan and /odom are in the `odom` frame and are transformed into the
robot frame using the live /odom pose before drawing.
"""

import time
from collections import deque, defaultdict

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from nav_msgs.msg import Path, Odometry

from g1_detection.detection import HumanXZPredictor


QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── palette (BGR) ──────────────────────────────────────────────────────────
BG = (24, 24, 28)
PANEL = (32, 33, 38)
GRID = (55, 57, 64)
TEXT = (235, 235, 235)
MUTED = (150, 152, 160)
ACCENT = (255, 176, 46)      # amber — plan / brand
GOOD = (90, 210, 120)        # green — NORMAL
WARN = (40, 200, 235)        # yellow — SLOW
STOP = (60, 60, 235)         # red — STOP
ROBOTC = (255, 200, 90)      # robot marker

# distinct, high-contrast track colors (BGR)
TRACK_COLORS = [
    (86, 180, 233), (0, 158, 115), (240, 228, 66), (213, 94, 0),
    (204, 121, 167), (0, 114, 178), (230, 159, 0), (120, 200, 120),
]


def track_color(tid: int):
    return TRACK_COLORS[tid % len(TRACK_COLORS)]


def cmd_color(cmd: str):
    c = (cmd or "").upper()
    if c.startswith("STOP"):
        return STOP
    if c.startswith("SLOW"):
        return WARN
    return GOOD


class VizNode(Node):
    def __init__(self):
        super().__init__("viz_node")

        # ── params ──
        self.declare_parameter("model_path", "yolo26n.pt")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 384)
        self.declare_parameter("color_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/g1/human_cmd")
        self.declare_parameter("out_w", 1600)
        self.declare_parameter("out_h", 720)
        self.declare_parameter("bev_range_m", 6.0)     # BEV shows this many metres ahead
        self.declare_parameter("record", True)
        self.declare_parameter("record_path", "")       # "" -> auto demo_<ts>.mp4
        self.declare_parameter("record_fps", 15.0)
        self.declare_parameter("title", "G1 Human-Aware Navigation")

        gp = lambda k: self.get_parameter(k).value
        self._color_topic = gp("color_topic")
        self._plan_topic = gp("plan_topic")
        self._odom_topic = gp("odom_topic")
        self.OUT_W = int(gp("out_w"))
        self.OUT_H = int(gp("out_h"))
        self.BEV_RANGE = float(gp("bev_range_m"))
        self._title = gp("title")

        self._predictor = HumanXZPredictor(
            model_path=gp("model_path"), conf=gp("conf"), imgsz=gp("imgsz"),
            use_tracking=True, draw=False, profile=False,
        )

        # ── perception state ──
        self._latest_depth = None
        self._fx = self._fy = self._cx = self._cy = None
        self._u_term = self._v_term = None
        self._center_px_hist = defaultdict(lambda: deque(maxlen=8))  # image-space motion

        # ── nav state (odom frame) ──
        self._plan_xy = []                       # [(x,y), ...] in odom
        self._odom = None                        # (x, y, yaw)
        self._odom_traj = deque(maxlen=600)      # traveled (x,y) in odom
        self._latest_cmd = "NORMAL_OPERATION"
        self._t_prev = time.time()
        self._speed = 0.0
        self._fps_ema = 0.0

        # ── subs ──
        self.create_subscription(Image, gp("color_topic"), self._color_cb, QOS_SENSOR)
        self.create_subscription(Image, gp("depth_topic"), self._depth_cb, QOS_SENSOR)
        self.create_subscription(CameraInfo, gp("camera_info_topic"), self._info_cb, QOS_SENSOR)
        self.create_subscription(Path, self._plan_topic, self._plan_cb, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self.create_subscription(String, gp("cmd_topic"), self._cmd_cb, 10)

        # ── pub ──
        self._pub = self.create_publisher(Image, "/viz/overlay", 5)

        # ── recorder ──
        self._writer = None
        if gp("record"):
            path = gp("record_path") or \
                f"/home/tracelab/Desktop/g1_slam/demo_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
            self._record_path = path
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(path, fourcc, float(gp("record_fps")),
                                           (self.OUT_W, self.OUT_H))
            if self._writer.isOpened():
                self.get_logger().info(f"Recording demo to {path}")
            else:
                self.get_logger().warn(f"Could not open VideoWriter at {path}; recording OFF")
                self._writer = None

        self.get_logger().info("viz_node ready (waiting for camera_info).")

    # ── camera plumbing (same math as detection_node) ──
    def _info_cb(self, msg: CameraInfo):
        if self._fx is not None:
            return
        self._fx, self._fy = msg.k[0], msg.k[4]
        self._cx, self._cy = msg.k[2], msg.k[5]

    @staticmethod
    def _to_np(msg: Image) -> np.ndarray:
        enc = msg.encoding
        if enc in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8"):
            dt = np.uint8
        elif enc == "16UC1":
            dt = np.uint16
        else:
            dt = np.float32
        return np.frombuffer(msg.data, dtype=dt).reshape(msg.height, msg.width, -1).squeeze()

    def _ensure_grid(self, h, w):
        if self._u_term is not None and self._u_term.shape == (h, w):
            return
        uu, vv = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        self._u_term = (uu - self._cx) / self._fx
        self._v_term = (vv - self._cy) / self._fy

    def _depth_cb(self, msg: Image):
        if self._fx is None:
            return
        d = self._to_np(msg).astype(np.float32)
        if msg.encoding == "16UC1":
            d /= 1000.0
        h, w = d.shape
        self._ensure_grid(h, w)
        xyz = np.empty((h, w, 3), np.float32)
        xyz[:, :, 0] = self._u_term * d
        xyz[:, :, 1] = self._v_term * d
        xyz[:, :, 2] = d
        self._latest_depth = xyz

    def _plan_cb(self, msg: Path):
        self._plan_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = (p.x, p.y, yaw)
        if not self._odom_traj or np.hypot(p.x - self._odom_traj[-1][0],
                                           p.y - self._odom_traj[-1][1]) > 0.02:
            self._odom_traj.append((p.x, p.y))
        v = msg.twist.twist.linear
        self._speed = float(np.hypot(v.x, v.y))

    def _cmd_cb(self, msg: String):
        self._latest_cmd = msg.data

    # ── main render tick ──
    def _color_cb(self, msg: Image):
        if self._latest_depth is None:
            return
        img = self._to_np(msg)
        enc = msg.encoding
        if enc == "rgb8":
            color = img[:, :, ::-1]
        elif enc == "rgba8":
            color = img[:, :, 2::-1]
        elif enc == "bgra8":
            color = img[:, :, :3]
        else:
            color = img
        color = np.ascontiguousarray(color)

        _, result = self._predictor.process_frame(
            color_frame=color, point_cloud_xyz=self._latest_depth,
            timestamp=time.time(), T_odom_camera=None,
        )
        dets = result.get("detections", [])
        cmd = result.get("agv_movement", {}).get("command", self._latest_cmd)

        now = time.time()
        dt = now - self._t_prev
        self._t_prev = now
        if dt > 0:
            self._fps_ema = 0.9 * self._fps_ema + 0.1 * (1.0 / dt) if self._fps_ema else 1.0 / dt

        canvas = self._compose(color, dets, cmd)

        out = Image()
        out.header = msg.header
        out.height, out.width = canvas.shape[:2]
        out.encoding = "bgr8"
        out.step = out.width * 3
        out.data = canvas.tobytes()
        self._pub.publish(out)
        if self._writer is not None:
            self._writer.write(canvas)

    # ── composition ──
    def _compose(self, color, dets, cmd):
        canvas = np.full((self.OUT_H, self.OUT_W, 3), BG, np.uint8)
        ego_w = int(self.OUT_W * 0.70)
        ego = self._render_ego(color, dets, cmd, ego_w, self.OUT_H)
        canvas[:, :ego_w] = ego

        px = ego_w
        panel = canvas[:, px:]
        panel[:] = PANEL
        bev_h = int(self.OUT_H * 0.66)
        panel[:bev_h] = self._render_bev(dets, panel.shape[1], bev_h)
        self._render_stats(panel, dets, cmd, bev_h)
        canvas[:, px:] = panel

        cv2.rectangle(canvas, (px, 0), (px + 2, self.OUT_H), GRID, -1)
        return canvas

    # ── egocentric panel ──
    def _render_ego(self, color, dets, cmd, w, h):
        ego = cv2.resize(color, (w, h))
        sx, sy = w / color.shape[1], h / color.shape[0]

        # danger tint if not clear
        cc = cmd_color(cmd)
        if not (cmd or "").upper().startswith("NORMAL"):
            tint = ego.copy()
            cv2.rectangle(tint, (0, 0), (w, h), cc, -1)
            ego = cv2.addWeighted(tint, 0.12, ego, 0.88, 0)

        for d in dets:
            tid = int(d["track_id"])
            x1, y1, x2, y2 = [int(v) for v in d["bbox_xyxy"]]
            x1, y1, x2, y2 = int(x1*sx), int(y1*sy), int(x2*sx), int(y2*sy)
            col = track_color(tid)
            dist = d.get("distance_to_agv_m", 0.0) or 0.0
            in_danger = (d.get("region_relative_to_agv", "") or "").lower().startswith("front") and dist < 3.8
            thick = 3 if in_danger else 2
            self._round_rect(ego, x1, y1, x2, y2, col, thick)

            # label chip
            lbl = f"ID {tid}   {dist:.1f} m"
            (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(ego, (x1, y1 - th - 12), (x1 + tw + 14, y1), col, -1)
            cv2.putText(ego, lbl, (x1 + 7, y1 - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2, cv2.LINE_AA)

            # image-space motion arrow from center history
            cx = int((x1 + x2) / 2); cy = int((y1 + y2) / 2)
            self._center_px_hist[tid].append((cx, cy))
            hist = self._center_px_hist[tid]
            if len(hist) >= 4:
                ox, oy = hist[0]
                vx, vy = cx - ox, cy - oy
                if vx * vx + vy * vy > 25:
                    ex, ey = int(cx + vx * 1.8), int(cy + vy * 1.8)
                    cv2.arrowedLine(ego, (cx, cy), (ex, ey), (255, 255, 255), 3,
                                    cv2.LINE_AA, tipLength=0.3)
                    cv2.arrowedLine(ego, (cx, cy), (ex, ey), col, 2, cv2.LINE_AA, tipLength=0.3)

        self._banner(ego, cmd, len(dets))
        self._chrome(ego, w, h)
        return ego

    def _round_rect(self, img, x1, y1, x2, y2, col, t):
        r = max(4, min(18, (x2 - x1) // 8))
        for (ax, ay, bx, by) in [(x1+r, y1, x2-r, y1), (x1+r, y2, x2-r, y2),
                                 (x1, y1+r, x1, y2-r), (x2, y1+r, x2, y2-r)]:
            cv2.line(img, (ax, ay), (bx, by), col, t, cv2.LINE_AA)
        for (cx0, cy0, a0) in [(x1+r, y1+r, 180), (x2-r, y1+r, 270),
                               (x1+r, y2-r, 90), (x2-r, y2-r, 0)]:
            cv2.ellipse(img, (cx0, cy0), (r, r), a0, 0, 90, col, t, cv2.LINE_AA)

    def _banner(self, img, cmd, n):
        c = (cmd or "NORMAL_OPERATION").upper()
        col = cmd_color(c)
        txt = {"STOP": "STOP", "SLOW_DOWN": "SLOW DOWN"}.get(c, "CLEAR")
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 1.1, 3)
        x, y = 24, img.shape[0] - 30
        cv2.rectangle(img, (x - 12, y - th - 20), (x + tw + 24, y + 16), (18, 18, 20), -1)
        cv2.rectangle(img, (x - 12, y - th - 20), (x - 4, y + 16), col, -1)
        cv2.putText(img, txt, (x + 8, y), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3, cv2.LINE_AA)

    def _chrome(self, img, w, h):
        cv2.rectangle(img, (0, 0), (w, 40), (18, 18, 20), -1)
        cv2.putText(img, self._title, (16, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, TEXT, 2, cv2.LINE_AA)
        cv2.putText(img, "EGOCENTRIC  |  ZED 2i", (w - 250, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, MUTED, 1, cv2.LINE_AA)

    # ── bird's-eye panel (robot-centric; forward = up) ──
    def _render_bev(self, dets, w, h):
        bev = np.full((h, w, 3), PANEL, np.uint8)
        cx, cy = w // 2, int(h * 0.86)              # robot at bottom-center
        scale = (h * 0.78) / self.BEV_RANGE          # px per metre

        def rel_to_px(fwd, left):
            return int(cx - left * scale), int(cy - fwd * scale)

        # range rings + labels
        for r in range(1, int(self.BEV_RANGE) + 1):
            cv2.circle(bev, (cx, cy), int(r * scale), GRID, 1, cv2.LINE_AA)
            cv2.putText(bev, f"{r}m", (cx + 4, cy - int(r * scale) + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, MUTED, 1, cv2.LINE_AA)
        cv2.line(bev, (cx, 30), (cx, cy), GRID, 1, cv2.LINE_AA)

        # danger rings (front stop 1.8m red, slow 3.8m yellow)
        self._arc(bev, cx, cy, 3.8 * scale, WARN)
        self._arc(bev, cx, cy, 1.8 * scale, STOP)

        # nav plan + traveled path (odom -> robot frame)
        if self._odom is not None:
            rx, ry, ryaw = self._odom
            c, s = np.cos(ryaw), np.sin(ryaw)

            def odom_to_px(x, y):
                dx, dy = x - rx, y - ry
                fwd = dx * c + dy * s
                left = -dx * s + dy * c
                return rel_to_px(fwd, left)

            if len(self._odom_traj) > 1:
                pts = np.array([odom_to_px(x, y) for x, y in self._odom_traj], np.int32)
                cv2.polylines(bev, [pts], False, (120, 120, 130), 2, cv2.LINE_AA)
            if len(self._plan_xy) > 1:
                pts = np.array([odom_to_px(x, y) for x, y in self._plan_xy], np.int32)
                cv2.polylines(bev, [pts], False, ACCENT, 3, cv2.LINE_AA)
                gx, gy = odom_to_px(*self._plan_xy[-1])
                cv2.circle(bev, (gx, gy), 7, ACCENT, -1, cv2.LINE_AA)

        # humans + predicted trails (camera-relative x=right, z=forward)
        for d in dets:
            tid = int(d["track_id"])
            col = track_color(tid)
            px, pz = d["position_xz_m"]["x"], d["position_xz_m"]["z"]
            hx, hy = rel_to_px(pz, -px)
            preds = d.get("predictions_xz", [])
            if len(preds) > 1:
                pts = [rel_to_px(p["z"], -p["x"]) for p in preds]
                for i in range(len(pts) - 1):
                    a = int(60 + 195 * i / max(1, len(pts) - 1))
                    cc = tuple(int(v) for v in col)
                    cv2.line(bev, pts[i], pts[i + 1], cc, 2, cv2.LINE_AA)
                cv2.arrowedLine(bev, pts[-2], pts[-1], col, 2, cv2.LINE_AA, tipLength=0.4)
            cv2.circle(bev, (hx, hy), 8, col, -1, cv2.LINE_AA)
            cv2.circle(bev, (hx, hy), 8, (245, 245, 245), 1, cv2.LINE_AA)
            cv2.putText(bev, str(tid), (hx + 10, hy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)

        # robot marker (triangle pointing up)
        tri = np.array([[cx, cy - 14], [cx - 10, cy + 8], [cx + 10, cy + 8]], np.int32)
        cv2.fillConvexPoly(bev, tri, ROBOTC, cv2.LINE_AA)

        cv2.rectangle(bev, (0, 0), (w, 30), (18, 18, 20), -1)
        cv2.putText(bev, "BIRD'S-EYE VIEW", (12, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT, 1, cv2.LINE_AA)
        cv2.rectangle(bev, (0, 0), (w - 1, h - 1), GRID, 1)
        return bev

    def _arc(self, img, cx, cy, r, col):
        overlay = img.copy()
        cv2.ellipse(overlay, (cx, cy), (int(r), int(r)), 0, 180, 360, col, -1, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.10, img, 0.90, 0, img)
        cv2.ellipse(img, (cx, cy), (int(r), int(r)), 0, 180, 360, col, 1, cv2.LINE_AA)

    # ── stats strip ──
    def _render_stats(self, panel, dets, cmd, y0):
        w = panel.shape[1]
        cv2.line(panel, (0, y0), (w, y0), GRID, 1)
        col = cmd_color(cmd)
        rows = [
            ("STATUS", (cmd or "NORMAL_OPERATION").replace("_", " ").title(), col),
            ("HUMANS TRACKED", str(len(dets)), TEXT),
            ("ROBOT SPEED", f"{self._speed:0.2f} m/s", TEXT),
            ("NEAREST", self._nearest_txt(dets), TEXT),
            ("PIPELINE", f"{self._fps_ema:0.1f} FPS", MUTED),
        ]
        y = y0 + 44
        for label, val, vcol in rows:
            cv2.putText(panel, label, (20, y - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, MUTED, 1, cv2.LINE_AA)
            cv2.putText(panel, val, (20, y + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, vcol, 2, cv2.LINE_AA)
            y += 52

    def _nearest_txt(self, dets):
        if not dets:
            return "--"
        d = min(dets, key=lambda x: x.get("distance_to_agv_m", 9e9) or 9e9)
        return f"ID {int(d['track_id'])}  @ {d.get('distance_to_agv_m', 0.0):.1f} m"

    def destroy_node(self):
        if self._writer is not None:
            self._writer.release()
            self.get_logger().info(f"Saved demo video: {self._record_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
