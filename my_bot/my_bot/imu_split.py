#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


class ImuSplitterNode(Node):
    def __init__(self):
        super().__init__('imu_splitter_node')

        # Subscribe to the IMU topic
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',  # Replace with your IMU topic name
            self.imu_callback,
            10
        )

        # Publishers for acceleration and angular velocity
        self.acceleration_publisher = self.create_publisher(Vector3, '/imu_acc', 10)
        self.angular_velocity_publisher = self.create_publisher(Vector3, '/imu_ang', 10)

        self.get_logger().info("IMU Splitter Node Initialized.")

    def imu_callback(self, msg: Imu):
        # Extract acceleration
        acc_msg = Vector3()
        acc_msg.x = msg.linear_acceleration.x
        acc_msg.y = msg.linear_acceleration.y
        acc_msg.z = msg.linear_acceleration.z

        # Publish acceleration
        self.acceleration_publisher.publish(acc_msg)
        self.get_logger().info(f"Published Acceleration: [{acc_msg.x, acc_msg.y, acc_msg.z}]")

        # Extract angular velocity
        ang_msg = Vector3()
        ang_msg.x = msg.angular_velocity.x
        ang_msg.y = msg.angular_velocity.y
        ang_msg.z = msg.angular_velocity.z

        # Publish angular velocity
        self.angular_velocity_publisher.publish(ang_msg)
        self.get_logger().info(f"Published Angular Velocity: [{ang_msg.x, ang_msg.y, ang_msg.z}]")


def main(args=None):
    rclpy.init(args=args)
    node = ImuSplitterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IMU Splitter Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
