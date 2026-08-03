import rclpy
from rclpy.node import Node
import serial

class CmdNode(Node):

    def __init__(self):
        super().__init__('cmd_node')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.get_logger().info("Type speed (-100 to 100) or 's' to stop")

        self.timer = self.create_timer(0.1, self.send_cmd)

    def send_cmd(self):
        try:
            cmd = input("CMD >> ")
            self.ser.write((cmd + '\n').encode())
        except Exception as e:
            self.get_logger().warn(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = CmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
