import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    nav_pkg = get_package_share_directory('rover_navigation')
    params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(nav_pkg, 'map', 'rgarden_map.yaml')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # --- Static TF: base_link → laser ---
    base2lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
    )

    # --- Map Server ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params, {'yaml_filename': map_yaml}],
        remappings=remappings,
    )

    # --- AMCL ---
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params],
        remappings=remappings,
    )

    # --- Lifecycle manager for localization ---
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 30.0,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # --- Nav2 navigation nodes (delayed to let AMCL start first) ---
    navigation_nodes = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[params],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[params],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[params],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[params],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'bond_timeout': 30.0,
                    'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'velocity_smoother',
                        'collision_monitor',
                        'bt_navigator',
                        'waypoint_follower',
                    ],
                }],
            ),
        ]
    )

    return LaunchDescription([
        base2lidar,
        map_server,
        amcl,
        lifecycle_localization,
        navigation_nodes,
    ])