#!/usr/bin/env python3

# Connects to BNO055 via I2C
# Reads orientation, angular velocity, acceleration
# Auto-calibrates on first run, saves calibration
# Publishes to /imu/data for sensor fusion

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu  # standard ROS2 IMU message type
from std_msgs.msg import String  # used for human-readable calibration status

import board                     # CircuitPython board abstraction for I2C pins
import adafruit_bno055           # Adafruit CircuitPython driver for BNO055
import time
import json
import os

# Path to calibration file stored in the repo's calibration/ folder.
# __file__ is this script, ../../../../ walks up to the project root.
# BNO055 has no onboard EEPROM so offsets must be saved externally -
# without this, the sensor loses calibration every power cycle.
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), '../../../../calibration/bno055_offsets.json')


class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        # /imu/data: primary topic consumed by nav2/EKF for sensor fusion
        # /imu/calibration_status: human-readable status for debugging
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.cal_pub = self.create_publisher(String, '/imu/calibration_status', 10)

        # Initialize I2C bus and BNO055 sensor.
        # Fatal error if sensor not found - no point running without hardware.
        try:
            i2c = board.I2C()
            self.sensor = adafruit_bno055.Adafruit_BNO055_I2C(i2c)
            self.get_logger().info('BNO055 connected via I2C')
        except Exception as e:
            self.get_logger().fatal(f'Failed to connect to BNO055: {e}')
            raise

        # Tracks whether we've saved calibration this session.
        # Prevents re-writing the file every loop once all values hit 3.
        self._calibrated = False
        self._load_calibration()

        # Publish IMU data at 0.02s interval
        self.create_timer(0.02, self._publish)

    # CALIBRATION 
    # The BNO055 runs its own internal sensor fusion (NDOF mode) but needs
    # calibration offsets to correct for hardware imperfections and local
    # magnetic environment. Per the article these must be loaded in CONFIG
    # mode before switching back to NDOF - skipping this causes a load error.
    # https://robofoundry.medium.com/lessons-learned-while-working-with-imu-sensor-ros2-and-raspberry-pi-a4fec18a7c7

    def _load_calibration(self):
        """
        Load previously saved calibration offsets from JSON file if it exists.
        """
        if not os.path.exists(CALIBRATION_FILE):
            # First run - sensor will self-calibrate as the rover moves around.
            # Once all 4 values reach 3, offsets are auto-saved for future boots.
            self.get_logger().warn('No calibration file found - sensor will self-calibrate')
            return

        try:
            with open(CALIBRATION_FILE, 'r') as f:
                offsets = json.load(f)

            # Switch to CONFIG mode - required to write offsets to the sensor.
            # Writing in NDOF (operational) mode will throw an error.
            self.sensor.mode = adafruit_bno055.CONFIG_MODE
            time.sleep(0.025)  # short delay for mode switch to settle

            self.sensor.offsets_accelerometer = tuple(offsets['accel'])
            self.sensor.offsets_gyroscope     = tuple(offsets['gyro'])
            self.sensor.offsets_magnetometer  = tuple(offsets['mag'])

            # Switch back to NDOF for full 9-DOF sensor fusion
            self.sensor.mode = adafruit_bno055.NDOF_MODE
            time.sleep(0.025)

            self.get_logger().info('Calibration offsets loaded')

        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')

    def _save_calibration(self):
        """
        Save current calibration offsets to JSON. Called automatically once fully calibrated.
        """
        try:
            os.makedirs(os.path.dirname(CALIBRATION_FILE), exist_ok=True)
            offsets = {
                'accel': list(self.sensor.offsets_accelerometer),
                'gyro':  list(self.sensor.offsets_gyroscope),
                'mag':   list(self.sensor.offsets_magnetometer),
            }
            with open(CALIBRATION_FILE, 'w') as f:
                json.dump(offsets, f)
            self.get_logger().info(f'Calibration saved to {CALIBRATION_FILE}')
            self._calibrated = True  # stop re-saving on subsequent loops
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')


    # PUBLISH 
    def _publish(self):
        """
        Read sensor and publish IMU data at 50Hz.
        """
        try:
            quat  = self.sensor.quaternion      # orientation as (w, x, y, z)
            gyro  = self.sensor.gyroscope       # angular velocity in rad/s
            accel = self.sensor.acceleration    # linear acceleration in m/s²
            cal   = self.sensor.calibration_status  # tuple: (sys, gyro, accel, mag) each 0-3
        except Exception as e:
            self.get_logger().error(f'Sensor read error: {e}')
            return

        # Publish calibration status so we can monitor with:
        # ros2 topic echo /imu/calibration_status
        cal_msg = String()
        cal_msg.data = f'sys:{cal[0]} gyro:{cal[1]} accel:{cal[2]} mag:{cal[3]}'
        self.cal_pub.publish(cal_msg)

        # Once all four calibration values reach 3 (fully calibrated),
        # save offsets so they survive the next power cycle
        if not self._calibrated and all(c == 3 for c in cal):
            self._save_calibration()

        # Sensor returns None on individual fields during warmup before
        # it has enough data for a valid reading - skip publishing until ready
        if None in (quat, gyro, accel):
            self.get_logger().warn('Sensor not ready yet - waiting for calibration')
            return

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'  # must match URDF link name in rover_description

        # BNO055 quaternion order is (w, x, y, z)
        # ROS2 sensor_msgs/Imu expects (x, y, z, w) — remap here
        msg.orientation.w = quat[0]
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        # Setting covariance[0] to -1 tells downstream nodes (nav2, EKF) that covariance is unknown. 
        # Setting to 0 - that means "perfect certainty" and will break sensor fusion.
        msg.orientation_covariance[0]         = -1.0
        msg.angular_velocity_covariance[0]    = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IMUDriver()
        rclpy.spin(node)  # keeps node alive, processing callbacks
    except KeyboardInterrupt:
        pass  # clean exit on Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()