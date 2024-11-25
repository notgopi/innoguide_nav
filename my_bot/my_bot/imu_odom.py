#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np


class ImuToOdomNode(Node):
    def __init__(self):
        super().__init__('imu_to_odom_node')

        # Subscriber to the IMU topic
        self.create_subscription(Vector3, '/imu_acc', self.imu_acc_callback, 10)
        self.create_subscription(Vector3, '/imu_ang', self.imu_ang_callback, 10)
        # Publisher for the Odometry topic
        self.odom_publisher = self.create_publisher(Odometry, '/imu_odom', 10)

        # Initialize state variables
        self.position = np.zeros(3)  # [x, y, z]
        self.velocity = np.zeros(3)  # [vx, vy, vz]
        self.orientation = [0.0, 0.0, 0.0, 0.0]  # [roll, pitch, yaw]
        self.prev_time = None
        self.theta = 0.0
        self.get_logger().info("IMU to Odom Node Initialized")

    def imu_acc_callback(self, msg : Vector3):
        self.acc = [msg.x, msg.y, msg.z]
        self.publish_odom()

    def imu_ang_callback(self, msg : Vector3):
        self.ang_v = [msg.x, msg.y, msg.z]
        self.publish_odom()

    def publish_odom(self):
        # Time calculation
        current_time = self.get_clock().now().to_msg()
        if self.prev_time is None:
            self.prev_time = current_time
            return
        dt = (current_time.sec - self.prev_time.sec) + \
             (current_time.nanosec - self.prev_time.nanosec) * 1e-9
        self.prev_time = current_time


        # Velocity and position integration
        self.velocity[0] += self.acc[0] * dt
        self.velocity[1] += self.acc[1] * dt
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt

        # Convert orientation to quaternion
        self.theta = self.ang_v[2] * dt

        # Create Odometry message

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        # Position
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]

        self.orientation = self.quaternion_from_euler(0, 0, self.theta)
        # Orientation
        odom_msg.pose.pose.orientation.x = self.orientation[0]
        odom_msg.pose.pose.orientation.y = self.orientation[1]
        odom_msg.pose.pose.orientation.z = self.orientation[2]
        odom_msg.pose.pose.orientation.w = self.orientation[3]

        # Velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.ang_v[2]

        # Publish Odometry message
        self.odom_publisher.publish(odom_msg)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            q = [0] * 4
            q[0] = cy * cp * sr - sy * sp * cr
            q[1] = sy * cp * sr + cy * sp * cr
            q[2] = sy * cp * cr - cy * sp * sr
            q[3] = cy * cp * cr + sy * sp * sr

            return q


def main(args=None):
    rclpy.init(args=args)
    imu_to_odom_node = ImuToOdomNode()

    try:
        rclpy.spin(imu_to_odom_node)
    except KeyboardInterrupt:
        imu_to_odom_node.get_logger().info("Node stopped by user.")
    finally:
        imu_to_odom_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
