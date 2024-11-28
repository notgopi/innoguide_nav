import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer

class DataSynchronizer(Node):
    def __init__(self):
        super().__init__('tf_synchronizer')

        # Subscribers for odom and scan
        self.odom_sub = Subscriber(self, Odometry, '/odom')
        self.scan_sub = Subscriber(self, LaserScan, '/scan')

        # Approximate time synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.odom_sub, self.scan_sub],
            queue_size=10,
            slop=0.1  # Allowable timestamp mismatch
        )
        self.sync.registerCallback(self.callback)

    def callback(self, odom : Odometry, scan : LaserScan):
        self.get_logger().info(f'Received synchronized messages: {odom.header.stamp}, {scan.header.stamp}')
        # Process synchronized data here

def main(args=None):
    rclpy.init(args=args)
    node = DataSynchronizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
