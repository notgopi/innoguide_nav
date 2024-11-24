import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32

# Robot parameters
WHEEL_RADIUS = 0.05  # 5 cm
WHEEL_BASE = 0.3     # 30 cm
TICKS_PER_REV = 1000 # Encoder ticks per wheel revolution

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # Initialize pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.last_time = self.get_clock().now()

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Encoder subscribers
        self.create_subscription(Int32, '/encoder_left', self.left_encoder_callback, 10)
        self.create_subscription(Int32, '/encoder_right', self.right_encoder_callback, 10)

        # Store encoder values
        self.left_ticks = 0
        self.right_ticks = 0

        self.get_logger().info("Wheel Odometry Node Started")

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data
        self.update_odometry()

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data
        self.update_odometry()

    def update_odometry(self):
        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Calculate distance traveled by each wheel
        left_distance = (self.left_ticks - self.prev_left_ticks) * (2 * 3.14159 * WHEEL_RADIUS / TICKS_PER_REV)
        right_distance = (self.right_ticks - self.prev_right_ticks) * (2 * 3.14159 * WHEEL_RADIUS / TICKS_PER_REV)
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # Compute linear and angular velocities
        linear_velocity = (left_distance + right_distance) / 2.0
        angular_velocity = (right_distance - left_distance) / WHEEL_BASE

        # Update pose
        self.x += linear_velocity * dt * math.cos(self.theta)
        self.y += linear_velocity * dt * math.sin(self.theta)
        self.theta += angular_velocity * dt

        # Publish odometry
        self.publish_odom(linear_velocity, angular_velocity)

    def quaternion_from_euler(roll, pitch, yaw):
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

    def publish_odom(self, linear_velocity, angular_velocity):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quaternion = self.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Set velocities
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Publish odometry
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
