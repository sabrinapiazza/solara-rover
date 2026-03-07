import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rover_navigation = get_package_share_directory('rover_navigation')
    params_file = os.path.join(rover_navigation, 'config', 'slam.yaml')

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidarNode',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'frame_id': 'laser',
                'angle_compensate': True,
            }]
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
    ])