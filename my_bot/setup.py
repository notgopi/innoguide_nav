import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gopinath',
    maintainer_email='gopinath@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "waypoint_follower_node = my_bot.follow_waypoints:main",
            "lidar_filter = my_bot.laser_filter:main",
            "wheel_odometry_node = my_bot.odom_pub:main",
            "imu_to_odom_node = my_bot.imu_odom:main",
            "imu_splitter_node = my_bot.imu_split:main",
        ],
    },
)
