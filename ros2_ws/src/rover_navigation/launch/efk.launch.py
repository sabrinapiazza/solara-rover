# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     rover_navigation = get_package_share_directory('rover_navigation')
#     params_file1 = os.path.join(rover_navigation, 'config', 'ekf.yaml')
#     params_file2 = os.path.join(rover_navigation, 'config', 'navsat_transform.yaml')


#     return LaunchDescription([
#         Node(
#             package='robot_localization',
#             executable='ekf_node',
#             name='ekf_filter_node',
#             output='screen',
#             parameters=[params_file1],
#             # arguments=['--ros-args', '--log-level', 'DEBUG'],
#         ),
#         Node(
#             package='robot_localization',
#             executable='navsat_transform_node',
#             name='navsat_transform',
#             output='screen',
#             parameters=[params_file2],
#              remappings=[
#                 ('imu/data', '/imu/data'),
#                 ('imu', 'imu/data'),
#                 ('gps/fix', '/gps/fix'),
#                 ('odometry/filtered', '/odometry/filtered'),
#             ]
#         ),
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='imu_static_tf',
#             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
#         ),
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='gps_static_tf',
#             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link']
#         ),
#     ])

    
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rover_navigation = get_package_share_directory('rover_navigation')
    params_file1 = os.path.join(rover_navigation, 'config', 'ekf.yaml')
    params_file2 = os.path.join(rover_navigation, 'config', 'navsat_transform.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_file1],
            remappings=[
                ('odometry/filtered', '/odom'),
            ]
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[params_file2],
            remappings=[
                ('imu/data', '/imu/data'),
                ('imu', 'imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odom'),
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link']
        ),
    ])