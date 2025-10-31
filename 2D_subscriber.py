#!/usr/bin/env python3
# lidar_plotter.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import math

class LidarPlotter(Node):
    def __init__(self):
        super().__init__('lidar_plotter')
        self.create_subscription(LaserScan, 'scan', self.callback, 10)
        self.get_logger().info('LidarPlotter gestartet (warte auf Daten)...')

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.scat = self.ax.scatter([], [], s=5, c=[], cmap='viridis')
        self.ax.set_xlim(-8, 8)
        self.ax.set_ylim(-8, 8)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        plt.title("Live LIDAR Scan (2D) – farbcodiert nach Distanz")

    def callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Ungültige Werte entfernen
        mask = np.isfinite(ranges)
        ranges = ranges[mask]
        angles = angles[mask]

        if len(ranges) == 0:
            return

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Farbstufen definieren (4-Band-Farben)
        # Rot, Gelb, Violett, Blau
        colors = []
        for r in ranges:
            if r < 1.0:
                colors.append('red')
            elif r < 2.5:
                colors.append('yellow')
            elif r < 4.0:
                colors.append('violet')
            else:
                colors.append('blue')

        self.scat.set_offsets(np.c_[x, y])
        self.scat.set_color(colors)

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.close('all')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
