import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from g1_robot.g1 import G1Robot
from g1_robot.robot_interface import RobotConfig, RobotCommand, RobotMode


class RobotNode(Node):
    SLOW_SCALE = 0.4  # reduce to 40% speed when human nearby

    def __init__(self):
        super().__init__("robot_node")

        self.declare_parameter("robot_ip",   "192.168.123.161")
        self.declare_parameter("robot_port", 8080)
        self.declare_parameter("net_iface",  "")  # host NIC on robot subnet; "" = auto

        config = RobotConfig(
            ip=self.get_parameter("robot_ip").value,
            port=self.get_parameter("robot_port").value,
            net_iface=self.get_parameter("net_iface").value,
        )

        self._robot = G1Robot()
        ok = self._robot.setup(config)
        if not ok:
            self.get_logger().error("G1 setup failed — check IP and SDK.")

        # Human detection override state
        self._human_override = False   # True = hard stop
        self._vel_scale = 1.0          # 1.0 = normal, 0.4 = slow

        # Human detection commands from detection node
        self.create_subscription(String, "/g1/human_cmd", self._human_cmd_cb, 10)

        # Velocity commands from Nav2
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)

        self.get_logger().info("Robot node ready.")

    def _human_cmd_cb(self, msg: String):
        cmd = msg.data.upper()

        if cmd == "STOP":
            if not self._human_override:
                self.get_logger().warn("Human too close — STOP")
            self._human_override = True
            self._vel_scale = 0.0
            self._robot.stop()

        elif cmd == "SLOW_DOWN":
            self._human_override = False
            self._vel_scale = self.SLOW_SCALE
            self.get_logger().info("Human nearby — SLOW DOWN")

        elif cmd == "NORMAL_OPERATION":
            if self._human_override or self._vel_scale < 1.0:
                self.get_logger().info("Path clear — NORMAL OPERATION")
            self._human_override = False
            self._vel_scale = 1.0

    def _cmd_vel_cb(self, msg: Twist):
        if self._human_override:
            return  # safety override — don't move

        scale = self._vel_scale
        cmd = RobotCommand(
            mode=RobotMode.WALK,
            vx=msg.linear.x  * scale,
            vy=msg.linear.y  * scale,
            wz=msg.angular.z * scale,
        )
        self._robot.send_command(cmd)

    def destroy_node(self):
        self._robot.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
