# Main coordinator script
# Subscribes to ROS2 /gps/fix topic (to get GPS coordinates)
# Subscribes to ROS2 /imu/data topic (to get IMU data)
# Reads all environmental sensors

# Combines sensor data + GPS + IMU into ML and ESRI MQTT topics
# Publishes to MQTT for Lenovo server:
#   - "ML"   topic: environment + light
#   - "ESRI" topic: thermal + gps + imu
# Runs on timer (every INTERVAL seconds)

# NOTICE: WORKING INSIDE PAHO-PUBLISHER VIRTUAL ENV, NOT REGULAR VENV 
# regular venv is for when you're not running MQTT

import paho.mqtt.client as mqtt
import json, time, yaml
from sensors import environment, light, thermal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu

# LOAD CONFIG
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

broker   = config["broker"]
topics   = config["topics"]
INTERVAL = config["collector"]["poll_interval"]
QoS      = 1


# MQTT CALLBACKS
def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] CONNACK rc={rc}")

def on_publish(client, userdata, mid):
    print(f"[MQTT] PUBACK received for PacketId={mid}")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"[MQTT] Unexpected disconnect (rc={rc}). Will auto-reconnect.")


# COLLECTOR NODE
class CollectorNode(Node):
    def __init__(self, mqtt_client):
        super().__init__('collector')
        self.mqtt_client = mqtt_client

        # Cache for latest GPS fix - None until first message arrives
        self.latest_gps = None

        # Cache for latest IMU data - None until first message arrives
        self.latest_imu = None

        # Subscribe to GPS topic published by gps_driver.py
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            10
        )

        # Subscribe to IMU topic published by imu_driver.py
        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            10
        )


        # Timer replaces time.sleep() - fires collect_and_publish() every INTERVAL seconds
        self.timer = self.create_timer(INTERVAL, self.collect_and_publish)

        self.get_logger().info(f"Collector started, publishing every {INTERVAL}s")

    def gps_callback(self, msg: NavSatFix):
        # Fires automatically whenever gps_driver.py publishes a new fix.
        self.latest_gps = {
            "latitude":  msg.latitude,
            "longitude": msg.longitude,
            "altitude":  msg.altitude,
        }

    def imu_callback(self, msg: Imu):
        # Fires automatically whenever imu_driver.py publishes a new reading.
        self.latest_imu = {
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }

    def collect_and_publish(self):
        # Fires every INTERVAL seconds - reads sensors and publishes to MQTT
        timestamp    = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        env_data     = environment.get_data()
        light_data   = light.get_data()
        thermal_data = thermal.get_data()

        # Fallback if no GPS fix has arrived yet
        gps_data = self.latest_gps if self.latest_gps is not None else {
            "latitude":  None,
            "longitude": None,
            "altitude":  None,
        }

        # Fallback if no IMU reading has arrived yet
        imu_data = self.latest_imu if self.latest_imu is not None else {
            "orientation":         {"x": None, "y": None, "z": None, "w": None},
            "angular_velocity":    {"x": None, "y": None, "z": None},
            "linear_acceleration": {"x": None, "y": None, "z": None},
        }

        # ML topic: environment + light data (NEED TO ADD ESP32 SOIL DATA)
        self.mqtt_client.publish(
            topics["ML"],
            json.dumps({
                "timestamp":   timestamp,
                "environment": env_data,
                "light":       light_data,
            }),
            qos=QoS
        )

        # ESRI topic: thermal + gps + imu data
        self.mqtt_client.publish(
            topics["ESRI"],
            json.dumps({
                "timestamp": timestamp,
                "thermal":   thermal_data,
                "gps":       gps_data,
                "imu":       imu_data,
            }),
            qos=QoS
        )

        self.get_logger().info(f"Published at {timestamp}. GPS: {gps_data} | IMU ready: {self.latest_imu is not None}")


# MAIN
def main():
    # MQTT setup - runs in background thread via loop_start()
    client = mqtt.Client(client_id=broker["client_id"], clean_session=broker["clean_session"])
    client.on_connect    = on_connect
    client.on_publish    = on_publish
    client.on_disconnect = on_disconnect

    client.will_set(topics["status"], json.dumps({"status": "offline"}), qos=QoS, retain=True)
    client.connect(host=broker["host"], port=broker["port"], keepalive=broker["keepalive"])
    client.loop_start()
    client.publish(topics["status"], json.dumps({"status": "online"}), qos=QoS, retain=True)

    # ROS2 takes over main thread
    rclpy.init()
    node = CollectorNode(mqtt_client=client)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[COLLECTOR] Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()