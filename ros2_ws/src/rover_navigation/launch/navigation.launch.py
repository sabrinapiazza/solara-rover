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
        'maps', 'rgarden_map.yaml'  # adjust to your actual map filename
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

    return LaunchDescription([ map_server, lifecycle_manager])