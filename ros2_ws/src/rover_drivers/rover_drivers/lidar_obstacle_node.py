import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_node")

        # --- Parameters you can tune later ---
        self.declare_parameter("stop_distance", 0.6)     # meters
        self.declare_parameter("forward_speed", 0.2)     # m/s
        self.declare_parameter("turn_speed", 0.8)        # rad/s
        self.declare_parameter("front_arc_deg", 20.0)    # +/- degrees

        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.front_arc = math.radians(float(self.get_parameter("front_arc_deg").value))

        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Ready: /scan -> /cmd_vel")

    def scan_cb(self, msg: LaserScan):
        # Convert angle (rad) to index into ranges[]
        def idx(theta):
            i = int(round((theta - msg.angle_min) / msg.angle_increment))
            return max(0, min(i, len(msg.ranges) - 1))

        # Sample points across the front arc
        samples = 11
        angles = [(-self.front_arc + k*(2*self.front_arc/(samples-1))) for k in range(samples)]
        front = []
        for a in angles:
            r = msg.ranges[idx(a)]
            if math.isinf(r) or math.isnan(r):
                continue
            front.append(r)

        front_min = min(front) if front else float("inf")

        # quick left vs right comparison (helps pick turn direction)
        left = []
        right = []
        for deg in [30, 45, 60]:
            r = msg.ranges[idx(math.radians(deg))]
            if not (math.isinf(r) or math.isnan(r)):
                left.append(r)
        for deg in [-30, -45, -60]:
            r = msg.ranges[idx(math.radians(deg))]
            if not (math.isinf(r) or math.isnan(r)):
                right.append(r)

        left_min = min(left) if left else float("inf")
        right_min = min(right) if right else float("inf")

        cmd = Twist()
        if front_min < self.stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed if left_min > right_min else -self.turn_speed
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = LidarObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
