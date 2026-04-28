
# Launches robot state publisher (URDF/TF tree)
# Launches all sensors (GPS, IMU, LiDAR, motor bridge, claw)
# Launches sensor fusion (EKF - combines GPS+IMU+wheels)
# Launches SLAM (mapping/localization)
# Launches Nav2 (autonomous navigation)

# Launches all the other launch files


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    lifecycle_configure = TimerAction(
            period=15.0,  # wait 15 seconds for SLAM to fully start
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                    output='screen'
                )
            ]
        )

    lifecycle_activate = TimerAction(
            period=20.0,  # wait 2 more seconds after configure
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                )
            ]
        )
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_drivers'), 'launch', 'sensors_launch.py')
        )
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_navigation'), 'launch', 'efk.launch.py')
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_navigation'), 'launch', 'slam.launch.py')
        )
    )

    return LaunchDescription([sensors, ekf, slam, lifecycle_configure, lifecycle_activate])

    # return LaunchDescription([sensors, ekf])