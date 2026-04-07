from setuptools import find_packages, setup

package_name = 'claw_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amrutha',
    maintainer_email='amrutha@example.com',
    description='ROS 2 claw driver package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'claw_driver = claw_driver.claw_driver:main',
            'deploy_sensor_client = claw_driver.deploy_sensor_client:main',
        ],
    },
)