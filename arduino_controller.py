import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        self.subscriber = self.create_subscription(String, 'cmd_topic', self.callback, 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info("Connected to Arduino")

    def callback(self, msg):
        self.ser.write(msg.data.encode())
        self.get_logger().info(f"Sent to Arduino: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
