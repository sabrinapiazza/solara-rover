# Main coordinator script
# Subscribes to ROS2 /gps/fix topic (to get GPS coordinates)
# DONE - Reads all environmental sensors
# PARTIALLY DONE - Combines sensor data + GPS location into JSON
# DONE - Publishes to MQTT for Lenovo server
# DONE - Runs on timer (e.g., every 5 seconds)

# NOTICE: WORKING INSIDE PAHO-MQTT VIRTUAL ENV, NOT REGULAR VENV

import paho.mqtt.client as mqtt
import json, time, yaml
from sensors import environment, light, thermal

# LOAD CONFIG 
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

broker    = config["broker"]
topics    = config["topics"]
INTERVAL  = config["collector"]["poll_interval"]
QoS       = 1


# MQTT CALLBACKS 
def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] CONNACK rc={rc}")

def on_publish(client, userdata, mid):
    print(f"[MQTT] PUBACK received for PacketId={mid}")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"[MQTT] Unexpected disconnect (rc={rc}). Will auto-reconnect.")

# MAIN 
def main():
    client = mqtt.Client(client_id=broker["client_id"], clean_session=broker["clean_session"])
    client.on_connect    = on_connect
    client.on_publish    = on_publish
    client.on_disconnect = on_disconnect

    client.will_set(topics["status"], json.dumps({"status": "offline"}), qos=QoS, retain=True)
    client.connect(host=broker["host"], port=broker["port"], keepalive=broker["keepalive"])
    client.loop_start()
    client.publish(topics["status"], json.dumps({"status": "online"}), qos=QoS, retain=True)

    print(f"[COLLECTOR] Publishing every {INTERVAL}s. Ctrl-C to stop.")
    try:
        while True:
            timestamp    = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            env_data     = environment.get_data()
            light_data   = light.get_data()
            thermal_data = thermal.get_data()

            client.publish(topics["environment"], json.dumps({"timestamp": timestamp, **env_data}),     qos=QoS)
            client.publish(topics["light"],       json.dumps({"timestamp": timestamp, **light_data}),   qos=QoS)
            client.publish(topics["thermal"],     json.dumps({"timestamp": timestamp, **thermal_data}), qos=QoS)
            client.publish(topics["all"],         json.dumps({
                "timestamp":   timestamp,
                "environment": env_data,
                "light":       light_data,
                "thermal":     thermal_data,
            }), qos=QoS)

            print(f"[COLLECTOR] Published @ {timestamp}")
            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("[COLLECTOR] Shutting down.")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()