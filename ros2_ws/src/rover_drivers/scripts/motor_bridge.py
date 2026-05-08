import rclpy
from rclpy.node import Node
import serial
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        # self.serial_port = serial.Serial(
        #     port='/dev/ttyACM0',
        #     baudrate=115200,
        #     timeout=1.0
        # )
        
        self.serial_port = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            timeout=1.0,
            dsrdtr=False,
            rtscts=False
        )

        self.wheel_base = 0.3
        self.wheel_radius = 0.05
        self.ticks_per_rev = 360
        self.max_speed = 255
        self.max_linear = 1.0
        self.max_angular = 2.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.create_timer(0.05, self.read_serial)
        self.get_logger().info('Motor bridge node started')

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed  = linear - (angular * self.wheel_base / 2)
        right_speed = linear + (angular * self.wheel_base / 2)

        left_pwm  = int((left_speed  / self.max_linear) * self.max_speed)
        right_pwm = int((right_speed / self.max_linear) * self.max_speed)

        left_pwm  = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        MIN_PWM = 80
        if 0 < left_pwm < MIN_PWM:    left_pwm  =  MIN_PWM
        if 0 < right_pwm < MIN_PWM:   right_pwm =  MIN_PWM
        if -MIN_PWM < left_pwm < 0:   left_pwm  = -MIN_PWM
        if -MIN_PWM < right_pwm < 0:  right_pwm = -MIN_PWM

        command = f'CMD:{left_pwm},{right_pwm}\n'
        self.serial_port.write(command.encode())
    self.get_logger().info(f'Got cmd_vel: linear={msg.linear.x}')

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            if 'ENC:' in line:
                enc_start = line.index('ENC:') + 4
                data = line[enc_start:].split(',')
                # if len(data) == 2:
                if len(data) >= 2:
                    try:
                        left_ticks  = int(data[0])
                        right_ticks = int(data[1])
                        self.update_odometry(left_ticks, right_ticks)
                    except ValueError:
                        pass

    def update_odometry(self, left_ticks, right_ticks):
        if self.last_left_ticks == 0 and self.last_right_ticks == 0:
            self.last_left_ticks  = left_ticks
            self.last_right_ticks = right_ticks
            return

        delta_left_ticks  = left_ticks  - self.last_left_ticks
        delta_right_ticks = right_ticks - self.last_right_ticks
        self.last_left_ticks  = left_ticks
        self.last_right_ticks = right_ticks

        delta_left  = (delta_left_ticks  / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        delta_right = (delta_right_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

        delta_dist  = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_base

        self.x     += delta_dist * math.cos(self.theta)
        self.y     += delta_dist * math.sin(self.theta)
        self.theta += delta_theta

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