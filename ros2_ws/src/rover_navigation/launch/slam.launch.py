import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rover_navigation = get_package_share_directory('rover_navigation')
    params_file = os.path.join(rover_navigation, 'config', 'slam_params.yaml')

    return LaunchDescription([
        Node(
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
    ])