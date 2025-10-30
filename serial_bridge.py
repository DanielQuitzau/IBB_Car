# IBB_Car/serial_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from IBB_Car.utils.serial_utils import SerialConnection
from IBB_Car.utils.message_parser import parse_arduino_message

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.port = self.get_parameter('port').value

        self.serial = SerialConnection(self.port)
        self.serial.connect()

        # Publisher für Sensorwerte
        self.dist_pub = self.create_publisher(Float32, '/distance', 10)
        self.ir_pub = self.create_publisher(Float32, '/ir_sensor', 10)

        # Subscriber für Motorbefehle
        self.cmd_sub = self.create_subscription(String, '/cmd_drive', self.cmd_callback, 10)

        # Timer für das Lesen der seriellen Daten
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        line = self.serial.read_line()
        if line:
            msg = parse_arduino_message(line)
            if msg:
                if 'distance' in msg:
                    self.dist_pub.publish(Float32(data=msg['distance']))
                    self.get_logger().info(f"Distance: {msg['distance']} cm")
                if 'ir' in msg:
                    self.ir_pub.publish(Float32(data=msg['ir']))
                    self.get_logger().info(f"IR: {msg['ir']}")

    def cmd_callback(self, msg):
        """Empfängt Motorbefehle z.B. 'M1600' oder 'S1500'"""
        self.serial.send(msg.data)
        self.get_logger().info(f"➡️ Gesendet an Arduino: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.serial.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
