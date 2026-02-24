# Main coordinator script
# Subscribes to ROS2 /gps/fix topic (to get GPS coordinates)
# Reads all environmental sensors
# Combines sensor data + GPS location into JSON
# Publishes to MQTT for Lenovo server
# Runs on timer (e.g., every 5 seconds)

# NOTICE: WORKING INSIDE PAHO-MQTT VIRTUAL ENV, NOT REGULAR VENV
import paho.mqtt.client as mqtt
import json

# READ FROM EACH ENV SENSOR
# pull from sensors/environment.py, sensors/light.py, sensors/thermal.py - its own topics 
# environment.py and light.py giving data in a dictionary 

# COMBINE SENSOR/ML-SPECIFIC DATA INTO JSON 



# MQTT PUBLISHER CLIENT ESTABLISHMENT AND SEND 
# 1. establish paho eclipse mqtt library
# 2a. send a CONNECT message to the mos1 broker, which responds with a CONNACK message and a status code
# 2b. ClientId, CleanSession flag, KeepAlive (everything else optional)
# 3a. upon connection, send a PUBLISH message with JSON payload
# 3b. PacketId, Topic Name, Quality of Service (QoS), Retain Flag, Payload, DUP Flag
# 3c. Once the broker receives the PUBLISH message, it is the responsibility of the broker to deliver the message to all subscribers. 


