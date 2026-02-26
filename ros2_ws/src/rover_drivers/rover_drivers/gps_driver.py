# Opens serial connection to NEO-M8N GPS
# Reads NMEA sentences
# Parses lat/lon/altitude
# Publishes to /gps/fix topic for Nav2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None


class GPSDriver(Node):

    def __init__(self):
        super().__init__('gps_driver')

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("frame_id", "gps_link")

        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.frame_id = self.get_parameter("frame_id").value

        self.fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.raw_pub = self.create_publisher(String, "/gps/nmea", 10)

        self.ser = None

        self.timer = self.create_timer(0.2, self.read_gps)

        self.get_logger().info("GPS Driver started")

    def connect_serial(self):
        if serial is None:
            self.get_logger().error("pyserial not installed.")
            return False

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self.get_logger().info(f"Connected to GPS on {self.port}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Could not open serial port: {e}")
            return False

    def read_gps(self):
        if self.ser is None:
            if not self.connect_serial():
                return

        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except Exception:
            return

        if not line:
            return

        # Publish raw NMEA
        self.raw_pub.publish(String(data=line))

        if "GGA" not in line:
            return

        parts = line.split(",")
        if len(parts) < 10:
            return

        try:
            lat = self.convert_to_degrees(parts[2], parts[3])
            lon = self.convert_to_degrees(parts[4], parts[5])
            altitude = float(parts[9]) if parts[9] else float("nan")
        except Exception:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = altitude

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(msg)

    def convert_to_degrees(self, value, hemisphere):
        if not value:
            return 0.0

        degrees = float(value[:2])
        minutes = float(value[2:])
        decimal = degrees + minutes / 60.0

        if hemisphere in ["S", "W"]:
            decimal *= -1.0

        return decimal


def main():
    rclpy.init()
    node = GPSDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()