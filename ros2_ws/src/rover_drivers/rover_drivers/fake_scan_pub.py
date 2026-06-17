import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FakeScanPub(Node):
    def __init__(self):
        super().__init__("fake_scan_pub")

        self.declare_parameter("scenario", "clear")  # clear, front_block, left_block, right_block
        self.scenario = self.get_parameter("scenario").value

        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.n = 720
        self.angle_inc = (self.angle_max - self.angle_min) / self.n

        self.get_logger().info(f"Fake /scan publishing scenario={self.scenario}")

    def tick(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_inc
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 12.0

        ranges = [3.0] * self.n  # open space default

        def set_arc(center_deg, half_width_deg, dist):
            center = math.radians(center_deg)
            half = math.radians(half_width_deg)
            for i in range(self.n):
                ang = msg.angle_min + i * msg.angle_increment
                # shortest angular distance
                d = abs((ang - center + math.pi) % (2*math.pi) - math.pi)
                if d <= half:
                    ranges[i] = dist

        if self.scenario == "front_block":
            set_arc(0, 15, 0.4)
        elif self.scenario == "left_block":
            set_arc(60, 15, 0.4)
        elif self.scenario == "right_block":
            set_arc(-60, 15, 0.4)

        msg.ranges = ranges
        msg.intensities = []
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeScanPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
