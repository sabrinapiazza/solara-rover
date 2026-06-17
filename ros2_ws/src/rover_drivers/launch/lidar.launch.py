from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Real LiDAR driver
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",   # this matches the logs you showed earlier
            name="rplidar",
            output="screen",
            parameters=[{
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 115200,      # try 256000 if your model needs it
                "frame_id": "laser",
                "angle_compensate": True,
            }],
        ),

        # Your obstacle node (subscribes to /scan)
        Node(
            package="rover_drivers",
            executable="lidar_obstacle_node",
            name="lidar_obstacle_node",
            output="screen",
        ),
    ])