import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.actions

from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'nav_display.rviz')
    world_path = os.path.join(get_package_share_directory(package_name), 'world/Inno_world.world')
    my_map_file = os.path.join(get_package_share_directory(package_name), 'maps', 'inno_map.yaml')
    params_file = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')

    lifecycle_nodes = ['map_server_node', 'amcl_node']

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','robosp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_robot'],
                        output='screen')

    lf_node =Node(
        package='my_bot',
        executable='lidar_filter',
        name='lidar_filter',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(get_package_share_directory(package_name), 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    map_server_node =Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True},
            {'yaml_filename': my_map_file}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])
    
    localize = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),'launch','localization_launch.py'
                )]), launch_arguments={'use_sim_time': 'true',
                                        'map': my_map_file,
                                        '/scan': '/filtered_scan',
                                        'params_file': params_file,}.items()
    )

    navigate = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py'
                )]), launch_arguments={'use_sim_time': 'true',
                                        'map': my_map_file,
                                        '/scan': '/filtered_scan',
                                        'params_file': params_file}.items()
    )
    
    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        lf_node,
        rviz_node,
        #robot_localization_node,
        #launch_world,
        #map_server_node,
        #amcl_node,
        #lifecycle_manager_node,
        localize,
        navigate,
    ])