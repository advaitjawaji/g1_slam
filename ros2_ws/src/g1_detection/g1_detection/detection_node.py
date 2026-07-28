from __future__ import annotations

import json
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import tf2_ros

from g1_detection.detection import HumanXZPredictor


QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        self.declare_parameter("model_path", "yolo26n.pt")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 384)
        self.declare_parameter("camera_frame", "d435_link")
        self.declare_parameter("map_frame", "map")
        # World frame the overlay projects in. Nav2 (nav2_params_hw.yaml) uses
        # global_frame `odom` on hardware, and the ZED-VIO wiring publishes no
        # `map` frame at all — so this defaults to odom, independent of
        # `map_frame` above which the legacy marker path still uses.
        self.declare_parameter("world_frame", "odom")
        # Camera OPTICAL frame, normally taken from camera_info's header.frame_id.
        # Override only if that frame is missing from the TF tree; an empty
        # string keeps the auto-detected value.
        self.declare_parameter("optical_frame", "")
        # Camera topic names — overridable so a RealSense/ZED naming mismatch
        # is a launch arg, not a code edit. Check `ros2 topic list | grep camera`.
        self.declare_parameter("color_topic",       "/camera/color/image_raw")
        self.declare_parameter("depth_topic",       "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")

        model_path  = self.get_parameter("model_path").value
        conf        = self.get_parameter("conf").value
        imgsz       = self.get_parameter("imgsz").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._map_frame    = self.get_parameter("map_frame").value
        self._world_frame  = self.get_parameter("world_frame").value
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic  = self.get_parameter("camera_info_topic").value

        self._predictor = HumanXZPredictor(
            model_path=model_path,
            conf=conf,
            imgsz=imgsz,
            use_tracking=True,
            draw=False,
            profile=False,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._latest_depth: np.ndarray | None = None

        # Camera intrinsics (filled from /camera/color/camera_info).
        # Needed to back-project the depth image into a full XYZ cloud so the
        # predictor can read each person's lateral (X) AND forward (Z) position.
        self._fx = self._fy = self._cx = self._cy = None
        self._u_term = None   # cached (u - cx)/fx grid
        self._v_term = None   # cached (v - cy)/fy grid
        # The frame camera_info is expressed in — this is the true OPTICAL frame
        # (x right, y down, z forward), which is what the back-projected cloud
        # lives in. Read from the message rather than hard-coded so a
        # RealSense/ZED naming difference never silently skews the projection.
        self._optical_frame: str | None = self.get_parameter("optical_frame").value or None
        self._optical_forced = self._optical_frame is not None

        self.create_subscription(Image,      color_topic, self._color_cb, QOS_SENSOR)
        self.create_subscription(Image,      depth_topic, self._depth_cb, QOS_SENSOR)
        self.create_subscription(CameraInfo, info_topic,  self._info_cb,  QOS_SENSOR)
        self.get_logger().info(
            f"Subscribed to color='{color_topic}' depth='{depth_topic}' info='{info_topic}'"
        )

        self._pub_markers = self.create_publisher(MarkerArray, "/humans/markers", 10)
        self._pub_cmd     = self.create_publisher(String, "/g1/human_cmd", 10)
        # Full per-frame Human_dtp result for the demo overlay: bounding boxes,
        # track IDs, world positions and predicted trajectories, tagged with the
        # stamp of the exact source frame so the renderer can draw on that frame
        # rather than on whatever arrived during inference.
        self._pub_dets    = self.create_publisher(String, "/g1/detections", 10)
        self._warned_no_world_tf = False

        self.get_logger().info("Detection node ready (waiting for camera_info).")

    def _info_cb(self, msg: CameraInfo):
        if self._fx is not None:
            return  # intrinsics are static — only need them once
        self._fx, self._fy = msg.k[0], msg.k[4]
        self._cx, self._cy = msg.k[2], msg.k[5]
        if not self._optical_forced:
            self._optical_frame = msg.header.frame_id or self._camera_frame
        self.get_logger().info(
            f"Got intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} "
            f"cx={self._cx:.1f} cy={self._cy:.1f} optical_frame='{self._optical_frame}'"
        )

    def _ensure_backproject_grid(self, h: int, w: int):
        """Pre-compute (u-cx)/fx and (v-cy)/fy once for the image size."""
        if self._u_term is not None and self._u_term.shape == (h, w):
            return
        us = np.arange(w, dtype=np.float32)
        vs = np.arange(h, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)
        self._u_term = (uu - self._cx) / self._fx
        self._v_term = (vv - self._cy) / self._fy

    @staticmethod
    def _ros_image_to_numpy(msg: Image) -> np.ndarray:
        enc = msg.encoding
        if enc in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8"):
            dtype = np.uint8           # RealSense color, ZED color (bgra8)
        elif enc == "16UC1":
            dtype = np.uint16          # RealSense depth (mm)
        else:
            dtype = np.float32         # 32FC1 — ZED depth (metres)
        arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, -1)
        return arr.squeeze()

    def _depth_cb(self, msg: Image):
        if self._fx is None:
            return  # no intrinsics yet — can't back-project

        depth_raw = self._ros_image_to_numpy(msg).astype(np.float32)
        # 16UC1 depth is in mm — convert to metres
        if msg.encoding == "16UC1":
            depth_raw /= 1000.0

        h, w = depth_raw.shape
        self._ensure_backproject_grid(h, w)

        # Full pinhole back-projection: X=(u-cx)Z/fx, Y=(v-cy)Z/fy, Z=depth
        xyz = np.empty((h, w, 3), dtype=np.float32)
        xyz[:, :, 0] = self._u_term * depth_raw
        xyz[:, :, 1] = self._v_term * depth_raw
        xyz[:, :, 2] = depth_raw
        self._latest_depth = xyz

    def _color_cb(self, msg: Image):
        if self._latest_depth is None:
            return

        img = self._ros_image_to_numpy(msg)
        enc = msg.encoding
        # Produce a contiguous 3-channel BGR image for YOLO/cv2
        if enc == "rgb8":
            color = img[:, :, ::-1]            # RGB  -> BGR
        elif enc == "bgr8":
            color = img
        elif enc == "rgba8":
            color = img[:, :, 2::-1]           # RGBA -> BGR (drop alpha)
        elif enc == "bgra8":
            color = img[:, :, :3]              # BGRA -> BGR (drop alpha) — ZED
        else:
            color = img
        color = np.ascontiguousarray(color)
        point_cloud_xyz = self._latest_depth

        T_map_camera   = self._get_camera_transform()
        T_world_optical = self._lookup_matrix(self._world_frame, self._optical_frame)

        t_start = time.perf_counter()
        _, result = self._predictor.process_frame(
            color_frame=color,
            point_cloud_xyz=point_cloud_xyz,
            timestamp=time.time(),
            T_odom_camera=T_map_camera,
            T_world_optical=T_world_optical,
        )

        self._publish_markers(result)
        self._publish_cmd(result)
        self._publish_detections(
            result, msg, (time.perf_counter() - t_start) * 1000.0,
            world_ok=T_world_optical is not None,
        )

    def _lookup_matrix(self, target_frame: str, source_frame: str):
        """4x4 homogeneous transform T_target_source, or None if TF is unavailable."""
        if not target_frame or not source_frame:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            tx, ty, tz = t.x, t.y, t.z
            qx, qy, qz, qw = q.x, q.y, q.z, q.w

            R = np.array([
                [1-2*(qy**2+qz**2),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
                [2*(qx*qy+qz*qw),   1-2*(qx**2+qz**2),   2*(qy*qz-qx*qw)],
                [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
            ], dtype=np.float64)

            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[:3, 3] = [tx, ty, tz]
            return T
        except Exception:
            return None

    def _get_camera_transform(self):
        return self._lookup_matrix(self._map_frame, self._camera_frame)

    def _publish_markers(self, result: dict):
        marker_array = MarkerArray()
        detections = result.get("detections", [])

        for det in detections:
            track_id = det["track_id"]
            pos = det["position_xz_m"]

            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "humans"
            m.id = track_id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = pos["x"]
            m.pose.position.y = 0.0
            m.pose.position.z = pos["z"]
            m.pose.orientation.w = 1.0
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 1.8
            m.color.r = 1.0
            m.color.a = 0.7
            m.lifetime.sec = 1

            marker_array.markers.append(m)

            # Prediction path
            preds = det.get("predictions_xz", [])
            if preds:
                path_marker = Marker()
                path_marker.header.frame_id = self._map_frame
                path_marker.header.stamp = m.header.stamp
                path_marker.ns = "human_predictions"
                path_marker.id = track_id + 10000
                path_marker.type = Marker.LINE_STRIP
                path_marker.action = Marker.ADD
                path_marker.scale.x = 0.05
                path_marker.color.g = 1.0
                path_marker.color.a = 0.8
                path_marker.lifetime.sec = 1
                for p in preds:
                    pt = Point()
                    pt.x = p["x"]
                    pt.y = 0.0
                    pt.z = p["z"]
                    path_marker.points.append(pt)
                marker_array.markers.append(path_marker)

        self._pub_markers.publish(marker_array)

    def _publish_detections(self, result: dict, src: Image, latency_ms: float, world_ok: bool):
        """Serialise the Human_dtp result for the overlay renderer.

        JSON on a std_msgs/String keeps this dependency-free (no custom .msg
        package to build). The payload is a few KB at most: N humans x 20
        prediction points.
        """
        if not world_ok and not self._warned_no_world_tf:
            self._warned_no_world_tf = True
            self.get_logger().warn(
                f"No TF '{self._world_frame}' <- '{self._optical_frame}'. Overlay will "
                "still draw boxes, but human world tracks/predictions stay empty."
            )

        dets = []
        for det in result.get("detections", []):
            dets.append({
                "id":    det["track_id"],
                "bbox":  det["bbox_xyxy"],
                "conf":  bool(det.get("track_confirmed", False)),
                "dist":  det.get("distance_to_agv_m"),
                "region": det.get("region_relative_to_agv"),
                "cam":   det.get("position_camera_m"),
                "world": det.get("position_world_m"),
                # Downsample the 20-step horizon; the line reads the same at 10.
                "pred":  [[p["x"], p["y"]] for p in det.get("predictions_world", [])][::2],
            })

        agv = result.get("agv_movement", {})
        payload = {
            "stamp":      {"sec": src.header.stamp.sec, "nsec": src.header.stamp.nanosec},
            "frame_id":   src.header.frame_id,
            "width":      src.width,
            "height":     src.height,
            "world_frame": self._world_frame if world_ok else None,
            "cmd":        (agv.get("command") or "normal_operation").upper(),
            "state":      agv.get("range_state"),
            "closest_m":  agv.get("closest_human_depth_m"),
            "latency_ms": round(latency_ms, 1),
            "detections": dets,
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._pub_dets.publish(msg)

    def _publish_cmd(self, result: dict):
        cmd = result.get("agv_movement", {}).get("command", "normal_operation")
        msg = String()
        msg.data = cmd.upper()
        self._pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
