from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('rover_navigation'),
        'config', 'nav2_params.yaml'
    )

    map_yaml = os.path.join(
        get_package_share_directory('rover_navigation'),
        'map', 'rgarden_map.yaml'  # adjust to your actual map filename
    )

    lidar = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'frame_id': 'laser',
                'angle_compensate': True,
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # try 256000 if this doesn't work
            }]
    )
    base2lidar = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}]
    )

    #SABRINA: heyyyy add all ur remaining nodes here for simple nav2 

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([ map_server, lifecycle_manager, lidar, base2lidar])