from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    rover_drivers = get_package_share_directory('rover_drivers')
    rover_navigation = get_package_share_directory('rover_navigation')

    ultrasonic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_drivers, 'launch', 'ultrasonicbridge_launch.py')
        )
    )

    costmap_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rover_navigation, 'launch', 'nav+ult.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([ultrasonic_launch, costmap_launch])