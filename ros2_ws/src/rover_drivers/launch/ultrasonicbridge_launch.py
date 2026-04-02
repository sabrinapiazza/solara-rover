import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rover_drivers = get_package_share_directory('rover_drivers')




    return LaunchDescription([
        Node(
            package='rover_drivers',
            executable='ultrasonic_bridge',
            name='ultrasonic_bridge',
            parameters=[
                {"port": "/dev/ttyACM0"},
            ],
            output='screen'
        ),
    ])

