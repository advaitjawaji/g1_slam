from __future__ import annotations

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

        self.create_subscription(Image,      color_topic, self._color_cb, QOS_SENSOR)
        self.create_subscription(Image,      depth_topic, self._depth_cb, QOS_SENSOR)
        self.create_subscription(CameraInfo, info_topic,  self._info_cb,  QOS_SENSOR)
        self.get_logger().info(
            f"Subscribed to color='{color_topic}' depth='{depth_topic}' info='{info_topic}'"
        )

        self._pub_markers = self.create_publisher(MarkerArray, "/humans/markers", 10)
        self._pub_cmd     = self.create_publisher(String, "/g1/human_cmd", 10)

        self.get_logger().info("Detection node ready (waiting for camera_info).")

    def _info_cb(self, msg: CameraInfo):
        if self._fx is not None:
            return  # intrinsics are static — only need them once
        self._fx, self._fy = msg.k[0], msg.k[4]
        self._cx, self._cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f"Got intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} "
            f"cx={self._cx:.1f} cy={self._cy:.1f}"
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
        dtype = np.uint8 if msg.encoding in ("rgb8", "bgr8") else \
                np.uint16 if msg.encoding == "16UC1" else np.float32
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

        color = self._ros_image_to_numpy(msg)
        if msg.encoding == "rgb8":
            color = color[:, :, ::-1]  # RGB → BGR for cv2/YOLO
        point_cloud_xyz = self._latest_depth

        T_map_camera = self._get_camera_transform()

        _, result = self._predictor.process_frame(
            color_frame=color,
            point_cloud_xyz=point_cloud_xyz,
            timestamp=time.time(),
            T_odom_camera=T_map_camera,
        )

        self._publish_markers(result)
        self._publish_cmd(result)

    def _get_camera_transform(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._camera_frame,
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
