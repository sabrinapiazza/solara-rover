import rclpy
from rclpy.node import Node

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.get_logger().info('Motor bridge node started')

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

// Reads HC-SR04 sensors (4 sensors)
// Trigger pulse, measure echo time
// Converts to distance in cm
// Returns obstacle distances