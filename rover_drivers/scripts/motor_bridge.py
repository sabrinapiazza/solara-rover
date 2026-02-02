# Serial connection between ROS2 and Pico
# Subscribes to /cmd_vel from Nav2 (velocity commands)
# Sends motor commands to Pico
# Receives encoder data from Pico
# Publishes /odom (odometry from wheel encoders)
# Also handles ultrasonic sensor data from Pico