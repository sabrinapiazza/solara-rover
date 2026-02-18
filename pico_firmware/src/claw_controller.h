#ifndef CLAW_CONTROLLER_H
#define CLAW_CONTROLLER_H

#include <iostream>
#include <string>
#include "Servo.h"
#include <Arduino.h>


// dummy pins --> replace with actual pins later 
#define BASE_SERVO_PIN 0
#define LINK1_SERVO_PIN 1
#define LINK2_SERVO_PIN 2
#define GRIPPER_SERVO_PIN 3
#define SHAFT_MOTOR_DIR_PIN 4
#define SHAFT_MOTOR_PWM_PIN 5
#define DRILL_MOTOR_DIR_PIN 6
#define DRILL_MOTOR_PWM_PIN 7

// Preset positions (angles in degrees)
#define BASE_DRILL_SIDE 0 //faces away from rover, default
#define BASE_PICKUP_SIDE 180 //assumption: wherever the sensor box is 

#define LINK1_STOWED 90 // adjust based on actual orientiation buuut for now this is link1 initial state
#define LINK1_DRILL_DEPLOY 45 //adjust based on actual orientation but for now this is the angle link1 needs to be at to deploy the drill 
#define LINK1_SENSOR_PICKUP 60 //needs to be to pick up sensors
#define LINK1_SENSOR_DEPLOY 45 //deploy sensor into the ground

#define LINK2_STOWED 90 //same thing as link1
#define LINK2_DRILL_DEPLOY 30 //just a reminder to myself to change these
#define LINK2_SENSOR_PICKUP 45 //this too 
#define LINK2_SENSOR_DEPLOY 30

//these are angles that the library will eventually convert to pulses 
#define GRIPPER_OPEN 0 //i also assume that this is the initial state 
#define GRIPPER_CLOSED 90 //adjust based on actual gripper 

#define LINK1_SENSOR_SLOT_1 60 //matching angle with link1 sensor pickup angle for now

// Timing constants (milliseconds)
#define SERVO_MOVE_DELAY 2000
#define BASE_ROTATE_DELAY 3000
#define GRIPPER_SETTLE_DELAY 500
#define SHAFT_FLIP_DELAY 1500  // calibrate for 180 degree rotation -- delete 
//can i do another version? 
#define DRILL_DURATION 5000



using namespace std;




// State machine
enum ClawState { //enumerations are like a class but for constants so not as powerful
  IDLE, //initial state 
  DRILLING, // drill deployment and operation
  MOVING_TO_PICKUP, //move to pick up sensor
  PICKING_UP_SENSOR, //close gripper to pick up sensor
  MOVING_TO_DEPLOY, //move to deploy sensor
  DEPLOYING_SENSOR, //sensor deployment (lowering and releasing)
  RETURNING_TO_STOW //back to inital state 
};

class ClawController {
private:
  //private variables and functions are only accessible within the class
  Servo base_servo;
  Servo link1_servo;
  Servo link2_servo;
  Servo gripper_servo;

  ClawState current_state; //what position the claw is in at that point

  // Low-level movement functions
  //uses the states and enums to understand where the arm is and move to the drill position acccordingly 
  void move_arm_to_drill_position(); 
  void move_arm_to_pickup_position(); 
  void move_arm_to_deploy_position();
  void move_to_stow_position(); //initial state
  void retract_arm();
  
  void rotate_base_to_pickup_side();
  void rotate_base_to_drill_side();
  
  void flip_shaft_to_drill_side();
  void flip_shaft_to_claw_side();
  
  void run_drill(int duration_ms);
  void close_gripper();
  void open_gripper();

  void execute_full_sequence();

  public:
    ClawController();
    void init();  // initialize hardware
    //void send_serial_response(String response);
    void handle_command(String command);
    ClawState get_state();
};




#endif