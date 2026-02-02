// Main program loop
// Initializes all subsystems
// Reads serial commands from Pi 5
// Calls motor/encoder/ultrasonic/claw functions
// Sends sensor data back to Pi 5 over serial

//RECIEVING DATA 
// only file that interacts with ros2 topics
// recieves converted cmd velocity from motor bridge
// call motor driver 

// Nav2 publishes /cmd_vel (ROS2 topic)
// motor_bridge.py subscribes to /cmd_vel
// motor_bridge converts to serial command: "CMD,0.5,0.2\n"
// Sends over USB serial to Pico
// main.cpp reads serial, parses command
// Calls motor_driver.setSpeed(left, right)
// Motors move

//SENDING DATA

// encoder.cpp counts wheel ticks
// main.cpp formats: "ODOM,1234,5678\n"
// Sends over serial to Pi
// motor_bridge.py receives, parses
// Publishes to /odom (ROS2 topic)


