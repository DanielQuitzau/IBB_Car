#!/usr/bin/env python3
# rplidar_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar
import math

class RPLidarPublisher(Node):
    def __init__(self):
        super().__init__('rplidar_publisher')

        # Parameter
        self.declare_parameter('serial_port', 'COM3')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # LIDAR starten
        self.get_logger().info(f'Starte RPLIDAR auf Port {port}...')
        self.lidar = RPLidar(port, baudrate=baud)
        self.lidar.start_motor()
        self.scan_iter = self.lidar.iter_scans()

        # Timer für regelmäßiges Publizieren
        self.timer = self.create_timer(0.05, self.publish_scan)  # ~20 Hz

    def publish_scan(self):
        try:
            scan = next(self.scan_iter)
        except Exception as e:
            self.get_logger().error(f'LIDAR Fehler: {e}')
            return

        msg = LaserScan()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = math.radians(1.0)
        msg.range_min = 0.02
        msg.range_max = 8.0

        ranges = [float('nan')] * 360
        for (_, angle, distance) in scan:
            deg = int(round(angle)) % 360
            ranges[deg] = distance / 1000.0  # mm → m

        msg.ranges = ranges
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Beende LIDAR-Verbindung...')
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
