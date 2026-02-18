//Controls multiple claw motors (gripper, joints, extension)
// Position control for claw servos/motors
// Reads claw encoder feedback
// Safety limits (force, position bounds)
#include "claw_controller.h"

ClawController::ClawController() {
  current_state = IDLE;
}

void ClawController::init() {
  // Setup motor driver pins
  pinMode(SHAFT_MOTOR_DIR_PIN, OUTPUT); //shaft motor controls flipping between drill and claw
  pinMode(SHAFT_MOTOR_PWM_PIN, OUTPUT); //speed 
  pinMode(DRILL_MOTOR_DIR_PIN, OUTPUT); //drilling action
  pinMode(DRILL_MOTOR_PWM_PIN, OUTPUT); //spped of drill 
  
  // Initialize motors to stopped state
  digitalWrite(SHAFT_MOTOR_DIR_PIN, LOW); //default direction
  analogWrite(SHAFT_MOTOR_PWM_PIN, 0); //stopped
  digitalWrite(DRILL_MOTOR_DIR_PIN, LOW); //default direction
  analogWrite(DRILL_MOTOR_PWM_PIN, 0); //stopped 
  
  // Attach servos to pins
  base_servo.attach(BASE_SERVO_PIN);
  link1_servo.attach(LINK1_SERVO_PIN);
  link2_servo.attach(LINK2_SERVO_PIN);
  gripper_servo.attach(GRIPPER_SERVO_PIN);
  
  // Move to safe stow position
  move_to_stow_position();
  gripper_servo.write(GRIPPER_OPEN);
}

void ClawController::handle_command(String command) {
  if (command == "DEPLOY_SENSOR") { 
    current_state = DRILLING;
    execute_full_sequence();
  } 
  else if (command == "STOW") {
    move_to_stow_position();
    current_state = IDLE;
  }
//   else if (command == "STATUS") {
//     // Send current state back over serial
//     //send_serial_response("STATE:" + current_state);
//   }
}

void ClawController::execute_full_sequence() {
  // Step 1: Drill
  //all functions are called in drilling, picking up sensor, and deploying sensor sequence 
  flip_shaft_to_drill_side(); //flip shaft to drill side first 
  move_arm_to_drill_position(); //move to appropriate drill angle 
  run_drill(DRILL_DURATION); //call function to start drill for specific duration 
  retract_arm(); //retract arm back to stow after drilling 

  //will most likely add more positions based on how the robot arm is configured; simplest design for now 
  
  // Step 2: Flip to pickup side
  flip_shaft_to_claw_side(); //flip shaft back to claw side
  rotate_base_to_pickup_side(); //rotate 180 degrees 
  
  // Step 3: Pickup sensor
  current_state = PICKING_UP_SENSOR; 
  move_arm_to_pickup_position(); //move to pickup positon 
  close_gripper(); //closes claw
  delay(GRIPPER_SETTLE_DELAY);
  retract_arm(); //not sure if i need this because i do not know how the physics is going to play out 
  //note for robotics: would retracting completely make the sensor drop? will it be able to turn suspended? 
  
  // Step 4: Deploy sensor
  current_state = DEPLOYING_SENSOR;
  rotate_base_to_drill_side(); //rotates back to the soil bed 
  move_arm_to_deploy_position(); //move to deploy position; probably have to adjust later for accuracy 
  open_gripper(); 
  delay(GRIPPER_SETTLE_DELAY);
  retract_arm();
  
  // Step 5: Return to stow
  current_state = RETURNING_TO_STOW; //current_status for testing 
  move_to_stow_position();
  current_state = IDLE;
  
//   send_serial_response("COMPLETE");
}

// Low-level movement implementations
void ClawController::move_arm_to_drill_position() {
  link1_servo.write(LINK1_DRILL_DEPLOY);
  link2_servo.write(LINK2_DRILL_DEPLOY);
  delay(SERVO_MOVE_DELAY);
}

void ClawController::move_arm_to_pickup_position() {
  link1_servo.write(LINK1_SENSOR_PICKUP); 
  link2_servo.write(LINK2_SENSOR_PICKUP);
  delay(SERVO_MOVE_DELAY);
}

void ClawController::move_arm_to_deploy_position() {
  // Same as drill position for now, adjust if needed
  link1_servo.write(LINK1_DRILL_DEPLOY); 
  link2_servo.write(LINK2_DRILL_DEPLOY);
  delay(SERVO_MOVE_DELAY);
}

void ClawController::move_to_stow_position() {
  link1_servo.write(LINK1_STOWED);
  link2_servo.write(LINK2_STOWED);
  base_servo.write(BASE_DRILL_SIDE); //move to drill side 
  delay(BASE_ROTATE_DELAY);
}

void ClawController::retract_arm() {
  move_to_stow_position();
}

void ClawController::rotate_base_to_pickup_side() {
  base_servo.write(BASE_PICKUP_SIDE);
  delay(BASE_ROTATE_DELAY);
}

void ClawController::rotate_base_to_drill_side() {
  base_servo.write(BASE_DRILL_SIDE);
  delay(BASE_ROTATE_DELAY);
}

void ClawController::flip_shaft_to_drill_side() {
  digitalWrite(SHAFT_MOTOR_DIR_PIN, 1); //
  analogWrite(SHAFT_MOTOR_PWM_PIN, 200); 
  delay(SHAFT_FLIP_DELAY);
  analogWrite(SHAFT_MOTOR_PWM_PIN, 0);
}

void ClawController::flip_shaft_to_claw_side() {
  digitalWrite(SHAFT_MOTOR_DIR_PIN, 0);   // reverse direction
  analogWrite(SHAFT_MOTOR_PWM_PIN, 200);    // set speed
  delay(SHAFT_FLIP_DELAY);
  analogWrite(SHAFT_MOTOR_PWM_PIN, 0);      // stop
}

void ClawController::run_drill(int duration_ms) {
  digitalWrite(DRILL_MOTOR_DIR_PIN, 1);
  analogWrite(DRILL_MOTOR_PWM_PIN, 255);
  delay(duration_ms);
  analogWrite(DRILL_MOTOR_PWM_PIN, 0);
}

void ClawController::close_gripper() {
  gripper_servo.write(GRIPPER_CLOSED);
  delay(GRIPPER_SETTLE_DELAY);
}

void ClawController::open_gripper() {
  gripper_servo.write(GRIPPER_OPEN);
  delay(GRIPPER_SETTLE_DELAY);
}

ClawState ClawController::get_state() {
  return current_state;
}