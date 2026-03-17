# Launches all sensor nodes at once
# Starts GPS driver
# Starts IMU driver
# Starts motor bridge


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rover_drivers = get_package_share_directory('rover_drivers')


    return LaunchDescription([
        Node(
            package='rover_drivers',
            executable='imu_driver',
            name='imu_driver',
            output='screen'
        ),
        Node(
            package='rover_drivers',
            executable='gps_driver',
            name='gps_driver',
            output='screen'
        ),
    ])

# just need to add motor_bridge