# Main coordinator script
# Subscribes to ROS2 /gps/fix topic (to get GPS coordinates)
# Reads all environmental sensors
# Combines sensor data + GPS location into JSON
# Publishes to MQTT for Lenovo server
# Runs on timer (every INTERVAL seconds)

# NOTICE: WORKING INSIDE PAHO-PUBLISHER VIRTUAL ENV, NOT REGULAR VENV

import paho.mqtt.client as mqtt
import json, time, yaml
from sensors import environment, light, thermal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

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

        # Subscribe to GPS topic published by gps_driver.py
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
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

    def collect_and_publish(self):
        # Fires every INTERVAL seconds - reads sensors and publishes to MQTT.
        timestamp    = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        env_data     = environment.get_data()
        light_data   = light.get_data()
        thermal_data = thermal.get_data()

        # Fallback if no GPS fix has arrived yet
        gps_data = self.latest_gps if self.latest_gps is not None else {
            "latitude": None,
            "longitude": None,
            "altitude": None,
        }

        self.mqtt_client.publish(topics["environment"], json.dumps({"timestamp": timestamp, **env_data}),     qos=QoS)
        self.mqtt_client.publish(topics["light"],       json.dumps({"timestamp": timestamp, **light_data}),   qos=QoS)
        self.mqtt_client.publish(topics["thermal"],     json.dumps({"timestamp": timestamp, **thermal_data}), qos=QoS)
        self.mqtt_client.publish(topics["gps"],         json.dumps({"timestamp": timestamp, **gps_data}),     qos=QoS)
        self.mqtt_client.publish(topics["all"],         json.dumps({"timestamp":   timestamp,
                                                                    "environment": env_data,
                                                                    "light":       light_data,
                                                                    "thermal":     thermal_data,}), qos=QoS)

        self.get_logger().info(f"Published at {timestamp}. GPS: {gps_data}")


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