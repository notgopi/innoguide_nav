from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    control_file = os.path.join(get_package_share_directory('my_bot'), 'config', 'ros_control_config.yaml')

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[control_file],
            output="screen"
        )
    ])