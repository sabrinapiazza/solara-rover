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

    costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        parameters=[params],
        output='screen'
    )

    # lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager',
    #     parameters=[params],
    #     output='screen'
    # )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'local_costmap'],  # map_server must come first
        }],
        output='screen'
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'] 
    )
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    base_to_ultrasonic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_ultrasonic_link',
        arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'ultrasonic_link']
        # adjust x, z offset to where the sensor actually sits on the chassis
    )

    return LaunchDescription([map_to_odom, odom_to_base, base_to_ultrasonic, map_server, costmap, lifecycle_manager])