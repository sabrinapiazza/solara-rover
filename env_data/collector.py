# Main coordinator script
# Subscribes to ROS2 /gps/fix topic (to get GPS coordinates)
# Reads all environmental sensors
# Combines sensor data + GPS location into JSON
# Publishes to MQTT for Lenovo server
# Runs on timer (e.g., every 5 seconds)