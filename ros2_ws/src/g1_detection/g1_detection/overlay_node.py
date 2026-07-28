"""
Demo overlay node — renders the egocentric ZED view with Human_dtp bounding
boxes, human trajectory forecasts, the Nav2 plan and the path actually walked,
then publishes it and (optionally) writes an MP4 for the demo reel.

    ros2 run g1_detection overlay_node --ros-args \
        -p color_topic:=/zed/zed_node/rgb/color/rect/image \
        -p camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info \
        -p record_path:=~/g1_demos/run1.mp4

Reads `/g1/detections` (published by detection_node) rather than running YOLO a
second time: the boxes on screen are exactly the ones the avoidance logic acted
on, and the Jetson only pays for one inference pass.

Rendering is driven by a fixed-rate timer, not by any input topic, so a stalled
camera or a detector restart degrades to a still frame instead of a gap in the
recording. Frames are matched to detections by source timestamp so the boxes sit
on the frame YOLO actually saw, not on whatever arrived during inference.
"""
from __future__ import annotations

import json
import os
import time
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import tf2_ros

from g1_detection import overlay_render as R


QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _fourcc(code: str) -> int:
    """OpenCV moved fourcc to VideoWriter.fourcc in 4.10; the old free function
    is still present but deprecated. Accept whichever this build provides."""
    fn = getattr(cv2.VideoWriter, "fourcc", None) or cv2.VideoWriter_fourcc
    return int(fn(*code))


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw_from_quat(x, y, z, w) -> float:
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _matrix_from_tf(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    M = np.eye(4, dtype=np.float64)
    M[:3, :3] = np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ], dtype=np.float64)
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def image_to_bgr(msg: Image) -> np.ndarray | None:
    """ROS Image -> contiguous BGR, without cv_bridge (NumPy 2 incompatibility)."""
    enc = msg.encoding
    if enc in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    else:
        return None
    if enc == "rgb8":
        out = arr[:, :, ::-1]
    elif enc == "bgr8":
        out = arr
    elif enc == "rgba8":
        out = arr[:, :, 2::-1]
    elif enc == "bgra8":
        out = arr[:, :, :3]
    else:
        out = cv2.cvtColor(arr.squeeze(), cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(out)


class OverlayNode(Node):
    def __init__(self):
        super().__init__("overlay_node")

        p = self.declare_parameter
        p("color_topic",       "/zed/zed_node/rgb/color/rect/image")
        p("camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info")
        p("detections_topic",  "/g1/detections")
        p("plan_topic",        "/plan")
        p("local_plan_topic",  "/local_plan")
        p("odom_topic",        "/odom")
        p("goal_topic",        "/goal_pose")
        p("output_topic",      "/g1/overlay/image")
        p("world_frame",       "odom")
        p("base_frame",        "pelvis")
        # The camera's OPTICAL frame (x right, y down, z forward). Normally read
        # from camera_info's header.frame_id, which is correct for both the ZED
        # and RealSense wrappers. Override only if that frame is absent from the
        # TF tree — e.g. a bare zed_wrapper Node that never loads the camera URDF
        # — in which case the AR layers silently stay off. Check with:
        #   ros2 run tf2_ros tf2_echo <frame> odom
        p("optical_frame",     "")
        # Floor height = base_frame's Z in the world frame, minus this. With ZED
        # VIO the odom origin sits at the camera's start pose (~1.2 m up), so the
        # floor is NOT world z=0 and the AR paths would otherwise render at eye
        # level. Set ground_z_mode:=fixed + ground_z to override.
        p("pelvis_height_m",   0.75)
        p("ground_z_mode",     "auto")     # auto | fixed
        p("ground_z",          0.0)
        p("render_rate_hz",    15.0)
        # Downscale before rendering. 0 = native ZED resolution. Rendering costs
        # ~16 ms/frame at 1280x720 on a desktop CPU; on the Jetson set this to
        # 960 (or drop render_rate_hz to 10) to leave headroom for YOLO.
        p("render_width",      0)
        p("detection_stale_s", 0.7)
        # Frames retained for stamp matching. Only needs to span the detector's
        # latency (~30-100 ms), and each 720p frame is ~2.8 MB — keep it small.
        p("frame_buffer",      12)
        p("trail_max_points",  4000)
        p("trail_min_step_m",  0.03)
        p("bev_range_m",       6.0)
        p("show_bev",          True)
        p("record",            True)
        p("record_path",       "")
        p("publish_compressed", True)
        p("title",             "UNITREE G1  |  AUTONOMOUS HUMAN AVOIDANCE")

        g = lambda k: self.get_parameter(k).value
        self._world_frame  = g("world_frame")
        self._base_frame   = g("base_frame")
        self._pelvis_h     = float(g("pelvis_height_m"))
        self._gz_mode      = g("ground_z_mode")
        self._gz_fixed     = float(g("ground_z"))
        self._stale_s      = float(g("detection_stale_s"))
        self._trail_max    = int(g("trail_max_points"))
        self._trail_step   = float(g("trail_min_step_m"))
        self._bev_range    = float(g("bev_range_m"))
        self._show_bev     = bool(g("show_bev"))
        self._rate         = float(g("render_rate_hz"))
        self._render_w     = int(g("render_width"))
        self._title        = g("title")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._frames: deque[tuple[int, np.ndarray]] = deque(
            maxlen=max(2, int(g("frame_buffer"))))
        self._intr: R.Intrinsics | None = None
        self._optical_frame: str | None = g("optical_frame") or None
        self._optical_forced = self._optical_frame is not None
        self._det: dict | None = None
        self._det_rx_t: float = 0.0
        self._plan: np.ndarray | None = None
        self._local_plan: np.ndarray | None = None
        self._goal: tuple[float, float] | None = None
        self._speed: float = 0.0
        self._trail: list[tuple[float, float]] = []
        self._t0 = time.time()
        self._render_times: deque[float] = deque(maxlen=30)
        self._warned_plan_frame = False

        self.create_subscription(Image, g("color_topic"), self._image_cb, QOS_SENSOR)
        self.create_subscription(CameraInfo, g("camera_info_topic"), self._info_cb, QOS_SENSOR)
        self.create_subscription(String, g("detections_topic"), self._det_cb, 10)
        self.create_subscription(Path, g("plan_topic"), self._plan_cb, 10)
        self.create_subscription(Path, g("local_plan_topic"), self._local_plan_cb, 10)
        self.create_subscription(Odometry, g("odom_topic"), self._odom_cb, QOS_SENSOR)
        self.create_subscription(PoseStamped, g("goal_topic"), self._goal_cb, 10)

        self._pub = self.create_publisher(Image, g("output_topic"), 2)
        self._pub_c = (self.create_publisher(CompressedImage, g("output_topic") + "/compressed", 2)
                       if bool(g("publish_compressed")) else None)

        self._writer: cv2.VideoWriter | None = None
        self._rec_t0 = 0.0
        self._rec_frames = 0
        self._record = bool(g("record"))
        self._record_path = os.path.expanduser(g("record_path") or "")
        if self._record and not self._record_path:
            self._record_path = os.path.expanduser(
                time.strftime("~/g1_demos/g1_demo_%Y%m%d_%H%M%S.mp4"))

        self.create_timer(1.0 / max(self._rate, 1.0), self._tick)
        self.get_logger().info(
            f"Overlay ready. color='{g('color_topic')}' dets='{g('detections_topic')}' "
            f"world='{self._world_frame}' -> {g('output_topic')}"
            + (f"  recording -> {self._record_path}" if self._record else "  (not recording)")
        )

    # ── Subscriptions ─────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        img = image_to_bgr(msg)
        if img is None:
            return
        self._frames.append((_stamp_ns(msg.header.stamp), img))

    def _info_cb(self, msg: CameraInfo):
        if self._intr is not None:
            return
        self._intr = R.Intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5],
                                  msg.width, msg.height)
        if not self._optical_forced:
            self._optical_frame = msg.header.frame_id
        self.get_logger().info(
            f"Intrinsics fx={msg.k[0]:.1f} {msg.width}x{msg.height} "
            f"optical_frame='{self._optical_frame}'"
            + ("  (overridden by param)" if self._optical_forced else ""))

    def _det_cb(self, msg: String):
        try:
            self._det = json.loads(msg.data)
            self._det_rx_t = time.time()
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(f"Bad detections payload: {exc}")

    def _path_to_xy(self, msg: Path) -> np.ndarray | None:
        if not msg.poses:
            return None
        if msg.header.frame_id and msg.header.frame_id != self._world_frame \
                and not self._warned_plan_frame:
            self._warned_plan_frame = True
            self.get_logger().warn(
                f"Plan is in '{msg.header.frame_id}' but world_frame is "
                f"'{self._world_frame}'. Paths will be drawn in the wrong place — "
                f"set world_frame to match Nav2's global_frame.")
        return np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses],
                        dtype=np.float64)

    def _plan_cb(self, msg: Path):
        self._plan = self._path_to_xy(msg)
        if self._plan is not None and self._goal is None and len(self._plan):
            self._goal = (float(self._plan[-1][0]), float(self._plan[-1][1]))

    def _local_plan_cb(self, msg: Path):
        self._local_plan = self._path_to_xy(msg)

    def _odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        self._speed = float(np.hypot(v.x, v.y))

    def _goal_cb(self, msg: PoseStamped):
        self._goal = (float(msg.pose.position.x), float(msg.pose.position.y))

    # ── Lookups ───────────────────────────────────────────────────────────

    def _lookup(self, target: str, source: str, stamp=None):
        if not target or not source:
            return None
        for t in ([stamp, rclpy.time.Time()] if stamp is not None else [rclpy.time.Time()]):
            try:
                return self._tf_buffer.lookup_transform(
                    target, source, t, timeout=rclpy.duration.Duration(seconds=0.02))
            except Exception:
                continue
        return None

    def _robot_pose(self):
        tf = self._lookup(self._world_frame, self._base_frame)
        if tf is None:
            return None, 0.0, None
        t, q = tf.transform.translation, tf.transform.rotation
        return (float(t.x), float(t.y)), _yaw_from_quat(q.x, q.y, q.z, q.w), float(t.z)

    def _push_trail(self, xy):
        if xy is None:
            return
        if not self._trail or np.hypot(xy[0] - self._trail[-1][0],
                                       xy[1] - self._trail[-1][1]) >= self._trail_step:
            self._trail.append((float(xy[0]), float(xy[1])))
            if len(self._trail) > self._trail_max:
                del self._trail[:len(self._trail) - self._trail_max]

    def _pick_frame(self, want_ns: int | None):
        """Frame whose stamp matches the detection, else the newest frame."""
        if not self._frames:
            return None
        if want_ns is None:
            return self._frames[-1][1]
        best, best_d = None, None
        for ns, img in self._frames:
            d = abs(ns - want_ns)
            if best_d is None or d < best_d:
                best, best_d = img, d
        # 60 ms — wider than a frame interval, tighter than visible box lag.
        return best if best_d is not None and best_d < 60_000_000 else self._frames[-1][1]

    # ── Render loop ───────────────────────────────────────────────────────

    def _build_humans(self, det: dict) -> list[R.Human]:
        out = []
        for d in det.get("detections", []):
            world = d.get("world") or None
            pred = d.get("pred") or []
            out.append(R.Human(
                track_id=int(d.get("id", -1)),
                bbox=d.get("bbox", [0, 0, 0, 0]),
                distance_m=d.get("dist"),
                region=d.get("region"),
                confirmed=bool(d.get("conf", False)),
                world_xy=None if world is None else (float(world["x"]), float(world["y"])),
                prediction_xy=(np.asarray(pred, dtype=np.float64).reshape(-1, 2)
                               if len(pred) >= 2 else None),
            ))
        return out

    def _tick(self):
        if self._intr is None or not self._frames:
            return

        det = self._det
        fresh = det is not None and (time.time() - self._det_rx_t) < self._stale_s
        want_ns = None
        if fresh:
            s = det.get("stamp") or {}
            if "sec" in s:
                want_ns = int(s["sec"]) * 1_000_000_000 + int(s["nsec"])

        img = self._pick_frame(want_ns)
        if img is None:
            return
        if self._render_w and img.shape[1] != self._render_w:
            scale = self._render_w / float(img.shape[1])
            img = cv2.resize(img, (self._render_w, int(round(img.shape[0] * scale))),
                             interpolation=cv2.INTER_AREA)
        h, w = img.shape[:2]

        robot_xy, yaw, base_z = self._robot_pose()
        self._push_trail(robot_xy)

        if self._gz_mode == "fixed" or base_z is None:
            ground_z = self._gz_fixed
        else:
            ground_z = base_z - self._pelvis_h

        T_opt_world = None
        if self._optical_frame:
            tf = self._lookup(self._optical_frame, self._world_frame)
            if tf is not None:
                T_opt_world = _matrix_from_tf(tf)

        # Detector saw the source resolution; we may render a resized copy.
        sx = w / float(det["width"]) if fresh and det.get("width") else 1.0
        sy = h / float(det["height"]) if fresh and det.get("height") else 1.0

        self._render_times.append(time.time())
        fps = 0.0
        if len(self._render_times) > 1:
            span = self._render_times[-1] - self._render_times[0]
            fps = (len(self._render_times) - 1) / span if span > 1e-6 else 0.0

        scene = R.Scene(
            image=img,
            intrinsics=self._intr,
            T_opt_world=T_opt_world,
            ground_z=ground_z,
            humans=self._build_humans(det) if fresh else [],
            cmd=(det.get("cmd") if fresh else "NORMAL_OPERATION") or "NORMAL_OPERATION",
            closest_human_m=det.get("closest_m") if fresh else None,
            plan_xy=self._plan,
            local_plan_xy=self._local_plan,
            travelled_xy=np.asarray(self._trail, dtype=np.float64) if len(self._trail) > 1 else None,
            goal_xy=self._goal,
            robot_xy=robot_xy,
            robot_yaw=yaw,
            speed_mps=self._speed,
            fps=fps,
            detect_latency_ms=det.get("latency_ms") if fresh else None,
            elapsed_s=time.time() - self._t0,
            title=self._title,
            bev_range_m=self._bev_range,
            show_bev=self._show_bev,
        )

        # Bounding boxes come in source-image pixels; scale if we resized.
        if fresh and (abs(sx - 1.0) > 1e-3 or abs(sy - 1.0) > 1e-3):
            for hm in scene.humans:
                hm.bbox = [hm.bbox[0] * sx, hm.bbox[1] * sy,
                           hm.bbox[2] * sx, hm.bbox[3] * sy]

        try:
            out = R.render(scene)
        except Exception as exc:                     # never kill the recording
            self.get_logger().warn(f"Render failed: {exc}", throttle_duration_sec=5.0)
            return

        self._publish(out)
        self._write(out)

    def _publish(self, bgr: np.ndarray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._optical_frame or "camera"
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = bgr.shape[1] * 3
        msg.data = bgr.tobytes()
        self._pub.publish(msg)

        if self._pub_c is not None:
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                c = CompressedImage()
                c.header = msg.header
                c.format = "jpeg"
                c.data = buf.tobytes()
                self._pub_c.publish(c)

    def _write(self, bgr: np.ndarray):
        if not self._record:
            return
        if self._writer is None:
            os.makedirs(os.path.dirname(self._record_path) or ".", exist_ok=True)
            self._writer = cv2.VideoWriter(
                self._record_path, _fourcc("mp4v"),
                self._rate, (bgr.shape[1], bgr.shape[0]))
            if not self._writer.isOpened():
                self.get_logger().error(f"Cannot open {self._record_path} — recording off.")
                self._record = False
                self._writer = None
                return
            self._rec_t0 = time.time()
            self._rec_frames = 0
            self.get_logger().info(f"Recording {bgr.shape[1]}x{bgr.shape[0]} -> {self._record_path}")

        # The file header fixes playback at render_rate_hz, but the render loop
        # may fall short of it (a busy Jetson). Repeat the frame to fill the gap
        # so the MP4's duration matches wall-clock instead of playing fast.
        due = int((time.time() - self._rec_t0) * self._rate) + 1
        n = int(np.clip(due - self._rec_frames, 1, self._rate))   # cap catch-up at 1 s
        for _ in range(n):
            self._writer.write(bgr)
        self._rec_frames += n

    def destroy_node(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            self.get_logger().info(f"Wrote {self._record_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()          # flushes the MP4
        if rclpy.ok():
            rclpy.shutdown()
