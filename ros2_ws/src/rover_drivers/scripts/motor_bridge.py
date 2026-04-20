import rclpy
from rclpy.node import Node
import serial
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        # Serial connection to Pico
        self.serial_port = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            timeout=1.0
        )

        # Rover physical properties -- UPDATE THESE TO MATCH ACTUAL HARDWARE
        self.wheel_base = 0.3        # distance between left and right wheels (meters)
        self.wheel_radius = 0.05     # radius of wheels (meters)
        self.ticks_per_rev = 360     # encoder ticks per full wheel revolution
        self.max_speed = 255         # max PWM value Pico accepts
        self.max_linear = 1.0        # max linear velocity (m/s)
        self.max_angular = 2.0       # max angular velocity (rad/s)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher -- publishes to /wheel/odom for EKF to fuse
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)

        # Timer to read serial at 20Hz
        self.create_timer(0.05, self.read_serial)

        self.get_logger().info('Motor bridge node started')

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive: convert linear + angular to left/right wheel speeds
        left_speed = linear - (angular * self.wheel_base / 2)
        right_speed = linear + (angular * self.wheel_base / 2)

        # Normalize to PWM range (-255 to 255)
        left_pwm = int((left_speed / self.max_linear) * self.max_speed)
        right_pwm = int((right_speed / self.max_linear) * self.max_speed)

        # Clamp to valid range
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        # Send to Pico -- left side controls front-left + rear-left
        #              -- right side controls front-right + rear-right
        command = f'CMD:{left_pwm},{right_pwm}\n'
        self.serial_port.write(command.encode())

    def read_serial(self):
    if self.serial_port.in_waiting > 0:
        line = self.serial_port.readline().decode().strip()
        # find ENC: anywhere in the line
        if 'ENC:' in line:
            enc_start = line.index('ENC:') + 4
            data = line[enc_start:].split(',')
            if len(data) == 2:
                try:
                    left_ticks = int(data[0])
                    right_ticks = int(data[1])
                    self.update_odometry(left_ticks, right_ticks)
                except ValueError:
                    pass
                
    def update_odometry(self, left_ticks, right_ticks):
        # How many ticks since last update
        delta_left_ticks = left_ticks - self.last_left_ticks
        delta_right_ticks = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        # Convert ticks to distance traveled (meters)
        delta_left = (delta_left_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        delta_right = (delta_right_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

        # Dead reckoning
        delta_dist = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_base

        self.x += delta_dist * math.cos(self.theta)
        self.y += delta_dist * math.sin(self.theta)
        self.theta += delta_theta

        # Publish wheel odometry -- EKF will fuse this with IMU and GPS
        now = self.get_clock().now()
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()