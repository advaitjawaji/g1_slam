import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from g1_robot.g1 import G1Robot
from g1_robot.robot_interface import RobotConfig, RobotCommand, RobotMode


class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_node")

        self.declare_parameter("robot_ip",   "192.168.123.161")
        self.declare_parameter("robot_port", 8080)

        config = RobotConfig(
            ip=self.get_parameter("robot_ip").value,
            port=self.get_parameter("robot_port").value,
        )

        self._robot = G1Robot()
        self._robot.setup(config)

        # Human detection commands (STOP / NORMAL_OPERATION)
        self.create_subscription(String, "/g1/human_cmd", self._human_cmd_cb, 10)

        # Nav2 velocity commands
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        self._human_override = False
        self.get_logger().info("Robot node ready.")

    def _human_cmd_cb(self, msg: String):
        if msg.data == "STOP":
            self._human_override = True
            self._robot.send_command(RobotCommand(mode=RobotMode.STOP))
        elif msg.data == "NORMAL_OPERATION":
            self._human_override = False

    def _cmd_vel_cb(self, msg: Twist):
        if self._human_override:
            return
        cmd = RobotCommand(
            mode=RobotMode.WALK,
            vx=msg.linear.x,
            vy=msg.linear.y,
            wz=msg.angular.z,
        )
        self._robot.send_command(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
