from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rover_drivers",
            executable="fake_scan_pub",
            name="fake_scan_pub",
            output="screen",
            parameters=[{"scenario": "front_block"}],  # try clear, left_block, right_block
        ),
        Node(
            package="rover_drivers",
            executable="lidar_obstacle_node",
            name="lidar_obstacle_node",
            output="screen",
        ),
    ])
