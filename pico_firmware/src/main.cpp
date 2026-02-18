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


#include <Servo.h>
#include "claw_controller.h"

ClawController claw;

void setup() {
  // Initialize serial for Pi 5 communication
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println("CLAW_READY");
  
  // Initialize claw hardware
  claw.init();
}

void loop() {
  // Check for commands from Pi 5 over serial
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    claw.handle_command(command);
  }
}