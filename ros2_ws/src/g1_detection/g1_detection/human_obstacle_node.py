"""
Converts detected human positions (from /humans/markers) into Nav2-compatible
people obstacles on /people so Nav2 routes around them.

Also subscribes to /humans/markers and republishes predicted trajectory
waypoints as lethal obstacles in the local costmap via /human_obstacle_cloud.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

QOS_SENSOR = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class HumanObstacleNode(Node):
    """
    Publishes detected humans as:
    1. /human_obstacle_cloud  — PointCloud of current + predicted positions
       injected into Nav2 obstacle layer for path replanning
    """

    def __init__(self):
        super().__init__("human_obstacle_node")

        self.declare_parameter("prediction_horizon_steps", 10)
        self.declare_parameter("map_frame", "map")

        self._pred_steps = self.get_parameter("prediction_horizon_steps").value
        self._map_frame  = self.get_parameter("map_frame").value

        self.create_subscription(
            MarkerArray, "/humans/markers", self._markers_cb, QOS_SENSOR
        )

        self._cloud_pub = self.create_publisher(
            PointCloud, "/human_obstacle_cloud", 10
        )

        self.get_logger().info("Human obstacle node ready.")

    def _markers_cb(self, msg: MarkerArray):
        cloud = PointCloud()
        cloud.header.frame_id = self._map_frame
        cloud.header.stamp = self.get_clock().now().to_msg()

        for marker in msg.markers:
            if marker.ns == "humans":
                # Current human position
                p = Point32()
                p.x = float(marker.pose.position.x)
                p.y = 0.0
                p.z = float(marker.pose.position.z)
                cloud.points.append(p)

            elif marker.ns == "human_predictions":
                # Predicted trajectory — add each point as an obstacle
                for pt in marker.points[:self._pred_steps]:
                    p = Point32()
                    p.x = float(pt.x)
                    p.y = 0.0
                    p.z = float(pt.z)
                    cloud.points.append(p)

        if cloud.points:
            self._cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = HumanObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
