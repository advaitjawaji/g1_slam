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

from cv_bridge import CvBridge
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

        model_path  = self.get_parameter("model_path").value
        conf        = self.get_parameter("conf").value
        imgsz       = self.get_parameter("imgsz").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._map_frame    = self.get_parameter("map_frame").value

        self._predictor = HumanXZPredictor(
            model_path=model_path,
            conf=conf,
            imgsz=imgsz,
            use_tracking=True,
            draw=False,
            profile=False,
        )

        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._latest_depth: np.ndarray | None = None

        self.create_subscription(Image, "/camera/color/image_raw",   self._color_cb, QOS_SENSOR)
        self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", self._depth_cb, QOS_SENSOR)

        self._pub_markers = self.create_publisher(MarkerArray, "/humans/markers", 10)
        self._pub_cmd     = self.create_publisher(String, "/g1/human_cmd", 10)

        self.get_logger().info("Detection node ready.")

    def _depth_cb(self, msg: Image):
        depth_m = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32) / 1000.0
        # Convert depth map to XYZ point cloud (Z only — x,y set to 0 as placeholder)
        # Full intrinsics-based projection can be added via CameraInfo subscription
        h, w = depth_m.shape
        xyz = np.zeros((h, w, 3), dtype=np.float32)
        xyz[:, :, 2] = depth_m
        self._latest_depth = xyz

    def _color_cb(self, msg: Image):
        if self._latest_depth is None:
            return

        color = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
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
