
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
    motors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_drivers'), 'launch', 'motors_launch.py')
        )
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

    # nav = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('rover_navigation'), 'launch', 'navigation.launch.py')
    #     )
    # )

    nav = TimerAction(
    period=10.0,
    actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rover_navigation'), 'launch', 'navigation.launch.py')
            )
        )
        ]
    ) 

    #timer to anticipate timing problems with nav2

    return LaunchDescription([sensors, motors, ekf, nav])

    # return LaunchDescription([sensors, ekf])