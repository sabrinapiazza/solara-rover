from setuptools import setup
import os
from glob import glob

package_name = 'rover_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SWE Team Tech',
    maintainer_email='swe.ucr@gmail.com',
    description='Hardware drivers for solara sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_driver = rover_drivers.gps_driver:main',
            'imu_driver = rover_drivers.imu_driver:main',
            'motor_bridge = rover_drivers.motor_bridge:main',
            'lidar_obstacle_node = rover_drivers.lidar_obstacle_node:main',
            'fake_scan_pub = rover_drivers.fake_scan_pub:main',
        ],
    },
)
