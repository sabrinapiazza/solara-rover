import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range  # standard ROS2 Range message type


# with serial.Serial('/dev/ttyS1', 19200, timeout=1) as ser:


class UltrasonicBridge(Node):

    def __init__(self):
        super().__init__('ultrasonic_bridge') # name of the node

        # Parameters
        self.declare_parameter("port", "/dev/pts/3") # default port, can be overridden by launch file or command line
        self.declare_parameter("baud", 9600) # default baud rate, can be overridden by launch file or command line
        self.declare_parameter("frame_id", "ultrasonic_link") # default frame_id for the Range message, can be overridden by launch file or command line

        self.port = self.get_parameter("port").value # e.g., "/dev/ttyS1"
        self.baud = self.get_parameter("baud").value # getting it from the parameter server
        self.frame_id = self.get_parameter("frame_id").value

        self.range_pub = self.create_publisher(Range, "/ultrasonic/fix", 10) # create a publisher for Range messages on the topic "/ultrasonic/fix" with a queue size of 10

        self.ser = None # will hold the serial connection once established

        self.create_timer(0.1, self.read_ultrasonic) # create a timer to call read_ultrasonic every 0.1 seconds (10 Hz)


        self.get_logger().info("Ultrasonic Bridge started") # log info message when the node starts

    def connect_serial(self):
        if serial is None:
            self.get_logger().error("pyserial not installed.")
            return False

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self.get_logger().info(f"Connected to Ultrasonic on {self.port}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Could not open serial port: {e}")
            return False

    def read_ultrasonic(self):
        if self.ser is None: # if we haven't established a serial connection yet, try to connect
            if not self.connect_serial():
                return

        try:
            line = self.ser.readline().decode(errors="ignore").strip() # read a line of data from the serial port, decode it to a string, ignore any decoding errors, and strip whitespace
        except Exception:
            return

        if not line: # if we didn't get any data, just return and wait for the next timer callback
            return
    
        try:
            distance = float(line) # try to convert the line of data to a float representing the distance in meters
        except ValueError:
            self.get_logger().warn(f"Could not parse line: {line}")
            return
        
        msg = Range() # create a new Range message

        msg.header.stamp = self.get_clock().now().to_msg() #clock 
        msg.header.frame_id = self.frame_id # e.g., "ultrasonic_link"
        msg.range = distance # distance from raw data 
        msg.radiation_type = Range.ULTRASOUND 
        msg.field_of_view = 0.26  # roughly 15 degrees in radians for HC-SR04
        msg.min_range = 0.02 # 2 cm minimum range for HC-SR04
        msg.max_range = 4.0 # 4 m maximum range for HC-SR04

        self.range_pub.publish(msg) # publish the Range message
        self.get_logger().debug(f"Published ultrasonic range: {distance:.2f} m")






def main():
    rclpy.init() # initialize ROS2
    node = UltrasonicBridge() # create the node instance
    try:
        rclpy.spin(node) # keep the node running to read and publish ultrasonic data
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


#ahhh