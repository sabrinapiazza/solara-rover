# Main coordinator script
# Subscribes to ROS2 /gps/fix topic (to get GPS coordinates)
# Subscribes to ROS2 /imu/data topic (to get IMU data)
# Reads all environmental sensors
# Combines sensor data + GPS + IMU into ML and ESRI MQTT topics
# Runs on timer (every INTERVAL seconds)

# NOTICE: WORKING INSIDE PAHO-PUBLISHER VIRTUAL ENV, NOT REGULAR VENV

# [TESTING MODE]
# - MQTT publishing is commented out
# - GPS and IMU are read directly from hardware (no ROS2 needed)
# - Data is printed to console instead of sent via MQTT
# - Environmental sensors (environment, light, thermal) work as normal
#
# TO SWITCH TO PRODUCTION:
# 1. Remove the [REMOVE FOR PRODUCTION] blocks
# 2. Uncomment all [UNCOMMENT FOR PRODUCTION] blocks
# 3. Run gps_driver.py and imu_driver.py as separate processes

import time, yaml
from sensors import environment, light, thermal

# [UNCOMMENT FOR PRODUCTION]
# import paho.mqtt.client as mqtt
# import json
# from sensor_msgs.msg import NavSatFix, Imu
# import rclpy
# from rclpy.node import Node

# [REMOVE FOR PRODUCTION] 
import board
import adafruit_bno055
import serial
import pynmea2


# LOAD CONFIG
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

# [UNCOMMENT FOR PRODUCTION]
# broker   = config["broker"]
# topics   = config["topics"]

INTERVAL = config["collector"]["poll_interval"]

# [UNCOMMENT FOR PRODUCTION] 
# QoS = 1


# [REMOVE FOR PRODUCTION] - Direct hardware reads for testing.
# In production, GPS and IMU data come from ROS2 subscriptions
# in CollectorNode (gps_callback / imu_callback).


def get_imu_data():
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    return {
        "orientation":         {"x": sensor.quaternion[1],    "y": sensor.quaternion[2],    "z": sensor.quaternion[3],    "w": sensor.quaternion[0]},
        "angular_velocity":    {"x": sensor.gyro[0],          "y": sensor.gyro[1],          "z": sensor.gyro[2]},
        "linear_acceleration": {"x": sensor.acceleration[0],  "y": sensor.acceleration[1],  "z": sensor.acceleration[2]},
    }

# def get_gps_data():
#     ser = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)
#     for _ in range(20):  # try up to 20 lines to find a valid fix
#         line = ser.readline().decode('ascii', errors='replace')
#         if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
#             msg = pynmea2.parse(line)
#             return {"latitude": msg.latitude, "longitude": msg.longitude, "altitude": msg.altitude}
#     return {"latitude": None, "longitude": None, "altitude": None}



# MQTT CALLBACKS
# [UNCOMMENT FOR PRODUCTION]
# def on_connect(client, userdata, flags, rc):
#     print(f"[MQTT] CONNACK rc={rc}")
#
# def on_publish(client, userdata, mid):
#     print(f"[MQTT] PUBACK received for PacketId={mid}")
#
# def on_disconnect(client, userdata, rc):
#     if rc != 0:
#         print(f"[MQTT] Unexpected disconnect (rc={rc}). Will auto-reconnect.")


# [REMOVE FOR PRODUCTION] - Standalone test loop.
# In production this is replaced by CollectorNode (ROS2 timer).

def run_test_loop():
    # Initialize IMU once - avoids re-initializing hardware every poll cycle
    i2c    = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    while True:
        timestamp    = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        # env_data     = environment.get_data()
        light_data   = light.get_data()
        # thermal_data = thermal.get_data()
        # gps_data     = get_gps_data()

        imu_data = {
            "orientation":         {"x": sensor.quaternion[1],   "y": sensor.quaternion[2],   "z": sensor.quaternion[3],   "w": sensor.quaternion[0]},
            "angular_velocity":    {"x": sensor.gyro[0],         "y": sensor.gyro[1],         "z": sensor.gyro[2]},
            "linear_acceleration": {"x": sensor.acceleration[0], "y": sensor.acceleration[1], "z": sensor.acceleration[2]},
        }

        # What would be sent to the ML MQTT topic
        print("\n========== ML TOPIC ==========")
        print(f"  timestamp:   {timestamp}")
        # print(f"  environment: {env_data}")
        print(f"  light:       {light_data}")

        # What would be sent to the ESRI MQTT topic
        print("\n========== ESRI TOPIC ==========")
        print(f"  timestamp: {timestamp}")
        # print(f"  thermal:   {thermal_data}")
        # print(f"  gps:       {gps_data}")
        print(f"  imu:       {imu_data}")
        print("================================\n")

        time.sleep(INTERVAL)



# COLLECTOR NODE
# [UNCOMMENT FOR PRODUCTION]
# class CollectorNode(Node):
#
#     def __init__(self, mqtt_client):
#         super().__init__('collector')
#         self.mqtt_client = mqtt_client
#
#         # Cache for latest GPS fix - None until first message arrives
#         self.latest_gps = None
#
#         # Cache for latest IMU data - None until first message arrives
#         self.latest_imu = None
#
#         # Subscribe to GPS topic published by gps_driver.py
#         self.gps_sub = self.create_subscription(
#             NavSatFix,
#             "/gps/fix",
#             self.gps_callback,
#             10
#         )
#
#         # Subscribe to IMU topic published by imu_driver.py
#         self.imu_sub = self.create_subscription(
#             Imu,
#             "/imu/data",
#             self.imu_callback,
#             10
#         )
#
#         # Timer replaces time.sleep() - fires collect_and_publish() every INTERVAL seconds
#         self.timer = self.create_timer(INTERVAL, self.collect_and_publish)
#
#         self.get_logger().info(f"Collector started, publishing every {INTERVAL}s")
#
#     def gps_callback(self, msg: NavSatFix):
#         self.latest_gps = {
#             "latitude":  msg.latitude,
#             "longitude": msg.longitude,
#             "altitude":  msg.altitude,
#         }
#
#     def imu_callback(self, msg: Imu):
#         self.latest_imu = {
#             "orientation":         {"x": msg.orientation.x,        "y": msg.orientation.y,        "z": msg.orientation.z,        "w": msg.orientation.w},
#             "angular_velocity":    {"x": msg.angular_velocity.x,   "y": msg.angular_velocity.y,   "z": msg.angular_velocity.z},
#             "linear_acceleration": {"x": msg.linear_acceleration.x,"y": msg.linear_acceleration.y,"z": msg.linear_acceleration.z},
#         }
#
#     def collect_and_publish(self):
#         timestamp    = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
#         env_data     = environment.get_data()
#         light_data   = light.get_data()
#         thermal_data = thermal.get_data()
#
#         gps_data = self.latest_gps if self.latest_gps is not None else {
#             "latitude":  None,
#             "longitude": None,
#             "altitude":  None,
#         }
#
#         imu_data = self.latest_imu if self.latest_imu is not None else {
#             "orientation":         {"x": None, "y": None, "z": None, "w": None},
#             "angular_velocity":    {"x": None, "y": None, "z": None},
#             "linear_acceleration": {"x": None, "y": None, "z": None},
#         }
#
#         # ML topic: environment + light
#         self.mqtt_client.publish(
#             topics["ML"],
#             json.dumps({
#                 "timestamp":   timestamp,
#                 "environment": env_data,
#                 "light":       light_data,
#             }),
#             qos=QoS
#         )
#
#         # ESRI topic: thermal + gps + imu
#         self.mqtt_client.publish(
#             topics["ESRI"],
#             json.dumps({
#                 "timestamp": timestamp,
#                 "thermal":   thermal_data,
#                 "gps":       gps_data,
#                 "imu":       imu_data,
#             }),
#             qos=QoS
#         )
#
#         self.get_logger().info(f"Published at {timestamp}. GPS: {gps_data} | IMU ready: {self.latest_imu is not None}")


# MAIN
def main():
    # [REMOVE FOR PRODUCTION] 
    run_test_loop()

    # [UNCOMMENT FOR PRODUCTION] 
    # client = mqtt.Client(client_id=broker["client_id"], clean_session=broker["clean_session"])
    # client.on_connect    = on_connect
    # client.on_publish    = on_publish
    # client.on_disconnect = on_disconnect
    #
    # client.will_set(topics["status"], json.dumps({"status": "offline"}), qos=QoS, retain=True)
    # client.connect(host=broker["host"], port=broker["port"], keepalive=broker["keepalive"])
    # client.loop_start()
    # client.publish(topics["status"], json.dumps({"status": "online"}), qos=QoS, retain=True)
    #
    # rclpy.init()
    # node = CollectorNode(mqtt_client=client)
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     print("[COLLECTOR] Shutting down.")
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()
    #     client.loop_stop()
    #     client.disconnect()


if __name__ == "__main__":
    main()