#!/usr/bin/env python3
import rclpy
from math import sin
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class LidarFilter(Node):

    def __init__(self):
        super().__init__("lidar_filter")
        self.laser_data = self.create_subscription(LaserScan, "/scan", self.filter_laser, 10)
        self.filter_data = self.create_publisher(LaserScan, "/filtered_scan", 10)

        self.threshold_dist = 0.35
        self.get_logger().info("Publishing the filtered_scan topic. Use RViz to visualize.")

    def filter_laser(self, msg : LaserScan):
        #angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        new_ranges = []
        for r in msg.ranges:
            if r > self.threshold_dist:
                new_ranges.append(r)
            else:
                new_ranges.append(np.inf)
        msg.ranges = new_ranges
        self.filter_data.publish(msg)


def main(args=None):
    rclpy.init(args = args)
    node = LidarFilter()
    rclpy.spin(node)
    rclpy.shutdown