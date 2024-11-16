#!/usr/bin/env python3
import rclpy
from math import sin
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

#define the required angles
theta_1 = (2 / 9) * np.pi
theta_2 = (5 / 18) * np.pi
theta_3 = (13 / 18) * np.pi
theta_4 = (7 / 9) * np.pi
theta_5 = (11 / 9) * np.pi
theta_6 = (23 / 18) * np.pi
theta_7 = (31 / 9) * np.pi
theta_8 = (16 / 9) * np.pi

class LidarFilter(Node):

    def __init__(self):
        super().__init__("lidar_filter")
        self.laser_data = self.create_subscription(LaserScan, "/scan", self.filter_laser, 10)
        self.filter_data = self.create_publisher(LaserScan, "/filtered_scan", 10)

        self.get_logger().info("Publishing the filtered_scan topic. Use RViz to visualize.")

    def filter_laser(self, msg : LaserScan):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        new_ranges = []
        for r, theta in zip(msg.ranges, angles):
            if theta >= theta_1 and theta <= theta_2:
                new_ranges.append(np.inf)
            elif theta >= theta_3 and theta <= theta_4:
                new_ranges.append(np.inf)
            elif theta >= theta_5 and theta <= theta_6:
                new_ranges.append(np.inf)
            elif theta >= theta_7 and theta <= theta_8:
                new_ranges.append(np.inf)
            else:
                new_ranges.append(r)
        #new_ranges = [r if abs(y) < self.extent else np.inf for r,y in zip(msg.ranges, points)]
        msg.ranges = new_ranges
        self.filter_data.publish(msg)


def main(args=None):
    rclpy.init(args = args)
    node = LidarFilter()
    rclpy.spin(node)
    rclpy.shutdown