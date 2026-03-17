# Launches all sensor nodes at once
# Starts LiDAR (using installed rplidar package)
# Starts GPS driver
# Starts IMU driver
# Starts motor bridge

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gps_node = Node(
        package="rover_drivers",
        executable="gps_driver",
        name="gps_driver",
        output="screen",
        parameters=[{
            "port": "/dev/ttyUSB0",
            "baud": 9600,
            "frame_id": "gps_link",
        }],
    )

    return LaunchDescription([gps_node])
