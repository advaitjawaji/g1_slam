"""
Converts detected human positions (from /humans/markers) into a PointCloud2 on
/human_obstacle_cloud that Nav2's ObstacleLayer can consume, so the planner
routes around current AND predicted human positions.

Why PointCloud2 (not PointCloud): Nav2's costmap observation sources only accept
``sensor_msgs/PointCloud2``. The old node published the deprecated
``sensor_msgs/PointCloud`` and was silently ignored.

⚠️  COORDINATE CONVENTION — READ BEFORE ENABLING IN nav2_params_hw.yaml.
``detection_node`` publishes human markers using a **Y-up** convention inherited
from Human_dtp's ZED pose: ``(x = lateral, y = 0, z = forward)``. A ROS costmap
/ RTAB-Map ``map`` frame is **Z-up** — the ground plane is X–Y and Z is height.
This node therefore remaps each marker's ground coordinates into the Z-up frame:

    cloud.x = marker.x          (lateral stays X)
    cloud.y = marker.z          (forward distance -> Y, the 2nd ground axis)
    cloud.z = obstacle height   (sampled across a band so the layer marks it)

That makes THIS node correct given a well-formed marker. BUT the upstream
transform in detection.py (`_camera_xz_to_odom_xz`, map rows 0+2) collapses the
human's lateral position in a true Z-up map frame — so the marker positions
themselves must be validated/fixed in the full stack before the costmap
injection can be trusted. Until then, leave the /human_obstacle_cloud source
commented out in nav2_params_hw.yaml. The reactive STOP path (/g1/human_cmd,
computed in camera frame) is unaffected by this and already works.
"""
from __future__ import annotations

from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class HumanObstacleNode(Node):
    """Republishes detected + predicted humans as a Nav2 obstacle PointCloud2."""

    def __init__(self):
        super().__init__("human_obstacle_node")

        self.declare_parameter("prediction_horizon_steps", 10)
        self.declare_parameter("map_frame", "map")
        # Footprint of each injected human (a small disk so the obstacle layer
        # reliably marks cells; inflation_layer handles the safety margin).
        self.declare_parameter("obstacle_radius_m", 0.3)
        # Height band (Z-up). Points are emitted across these so they fall inside
        # the costmap's [min_obstacle_height, max_obstacle_height] window.
        self.declare_parameter("height_min_m", 0.2)
        self.declare_parameter("height_max_m", 1.5)
        self.declare_parameter("height_layers", 3)

        self._pred_steps = int(self.get_parameter("prediction_horizon_steps").value)
        self._map_frame  = self.get_parameter("map_frame").value
        self._radius     = float(self.get_parameter("obstacle_radius_m").value)
        self._h_min      = float(self.get_parameter("height_min_m").value)
        self._h_max      = float(self.get_parameter("height_max_m").value)
        self._h_layers   = max(1, int(self.get_parameter("height_layers").value))

        # Precompute the vertical sample heights and the planar footprint offsets.
        if self._h_layers == 1:
            self._heights = [0.5 * (self._h_min + self._h_max)]
        else:
            step = (self._h_max - self._h_min) / (self._h_layers - 1)
            self._heights = [self._h_min + i * step for i in range(self._h_layers)]
        r = self._radius
        self._offsets = [(0.0, 0.0), (r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)]

        self.create_subscription(
            MarkerArray, "/humans/markers", self._markers_cb, QOS_SENSOR
        )
        self._cloud_pub = self.create_publisher(PointCloud2, "/human_obstacle_cloud", 10)

        self.get_logger().info("Human obstacle node ready (PointCloud2 / Z-up remap).")

    def _emit_column(self, points: list, gx: float, gy: float):
        """Append a small footprint of points (Z-up) at ground pos (gx, gy)."""
        for dx, dy in self._offsets:
            for h in self._heights:
                points.append((gx + dx, gy + dy, h))

    def _markers_cb(self, msg: MarkerArray):
        points: list[tuple[float, float, float]] = []

        for marker in msg.markers:
            if marker.ns == "humans":
                # Y-up marker (x=lateral, z=forward) -> Z-up ground (x, y=z).
                self._emit_column(points, float(marker.pose.position.x),
                                           float(marker.pose.position.z))
            elif marker.ns == "human_predictions":
                for pt in marker.points[:self._pred_steps]:
                    self._emit_column(points, float(pt.x), float(pt.z))

        if not points:
            return

        header = Header()
        header.frame_id = self._map_frame
        header.stamp = self.get_clock().now().to_msg()
        self._cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = HumanObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
