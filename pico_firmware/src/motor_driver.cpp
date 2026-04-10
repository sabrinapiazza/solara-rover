// Controls BTM7960 motor drivers for wheels
// PWM output to drive motors forward/backward
// Speed control functions
// Motor enable/disable
 
#define L_RPWM_PIN   2
#define L_LPWM_PIN   3
#define L_REN_PIN    4
#define L_LEN_PIN    5


#include "motor_driver.h"
#include <cmath>

/**
 * BTS7960 Motor Driver Class Implementation
 * 
 * The BTS7960 is a dual-channel motor driver that can control DC motors
 * in both forward and reverse directions using PWM (Pulse Width Modulation).
 * 
 * How it works:
 * - RPWM (Right PWM): Controls forward direction speed
 * - LPWM (Left PWM): Controls reverse direction speed  
 * - REN/LEN (Right/Left Enable): Turn on/off the respective channels
 * 
 * Safety Note: Only one PWM channel should be active at a time!
 * Running both RPWM and LPWM simultaneously can damage the motor or driver.
 */

/**
 * Constructor: Set up the pin connections for the motor driver
 * 
 * @param rpwm: Pin number for Right PWM (forward direction control)
 * @param lpwm: Pin number for Left PWM (reverse direction control)
 * @param ren:  Pin number for Right Enable (forward channel on/off)
 * @param len:  Pin number for Left Enable (reverse channel on/off)
 */
BTS7960Motor::BTS7960Motor(uint8_t rpwm, uint8_t lpwm, uint8_t ren, uint8_t len) {
  // Store the pin numbers for later use
  _rpwm = rpwm;
  _lpwm = lpwm;
  _ren = ren;
  _len = len;
  
  // Set default PWM range (0-1000 gives good resolution)
  _range = 1000;
  
  // Start with motor disabled for safety
  _enabled = false;
}

/**
 * Initialize the motor driver hardware
 * 
 * This function sets up all the pins and PWM settings needed to control the motor.
 * Call this once in your setup() function.
 * 
 * @param pwmFreqHz: PWM frequency in Hz (typical: 1000-20000 Hz)
 * @param pwmRange:  PWM resolution (1000 = 0.1% resolution, 255 = Arduino standard)
 */
void BTS7960Motor::begin(uint32_t pwmFreqHz, uint16_t pwmRange) {
  // Store the PWM range for later calculations
  _range = pwmRange;
  
  // === SET UP ENABLE PINS ===
  // These pins turn the motor driver channels on/off
  pinMode(_ren, OUTPUT);  // Right channel enable (forward)
  pinMode(_len, OUTPUT);  // Left channel enable (reverse)
  
  // Start with both channels disabled (safety first!)
  digitalWrite(_ren, LOW);
  digitalWrite(_len, LOW);
  _enabled = false;
  
  // === SET UP PWM CONFIGURATION ===
  // Configure the PWM frequency and resolution for smooth motor control
  analogWriteFreq(pwmFreqHz);  // How fast the PWM switches (affects motor smoothness)
  analogWriteRange(_range);    // PWM resolution (higher = more precise speed control)
  
  // === SET UP PWM PINS ===
  // These pins control the actual motor speed
  pinMode(_rpwm, OUTPUT);  // Forward direction PWM
  pinMode(_lpwm, OUTPUT);  // Reverse direction PWM
  
  // === SAFETY: START WITH MOTOR STOPPED ===
  // Set both PWM outputs to 0 so motor doesn't move unexpectedly
  analogWrite(_rpwm, 0);
  analogWrite(_lpwm, 0);
}

/**
 * Enable or disable the motor driver
 * 
 * When disabled, the motor can freely spin (coast mode).
 * When enabled, the motor is ready to receive commands.
 * 
 * @param on: true to enable motor control, false to disable
 */
void BTS7960Motor::enable(bool on) {
  _enabled = on;
  
  // Turn on/off both enable pins
  // HIGH = channel is active and ready to drive motor
  // LOW = channel is disabled (high impedance, motor can coast)
 if (on) { // Turn on both motor driver channels 
digitalWrite(_ren, HIGH); // Enable forward direction channel digitalWrite(_len, HIGH); // Enable reverse direction channel } 
else { // Turn off both motor driver channels 
digitalWrite(_ren, LOW); // Disable forward direction channel digitalWrite(_len, LOW); // Disable reverse direction channel
 }
  
  // Safety: If disabling, make sure no PWM is being sent
  if (!on) {
    analogWrite(_rpwm, 0);
    analogWrite(_lpwm, 0);
  }
}

/**
 * Helper function: Safely write PWM value to a pin
 * 
 * This prevents accidentally sending PWM values that are too high,
 * which could damage the motor or cause unexpected behavior.
 * 
 * @param pin:  Which pin to write to
 * @param duty: PWM duty cycle (0 to _range)
 */
void BTS7960Motor::writePWM(uint8_t pin, uint16_t duty) {
  // Clamp the duty cycle to prevent overflow
  if (duty > _range) duty = _range;
  
  // Send the PWM signal to the pin
  analogWrite(pin, duty);
}

/**
 * Main motor control function
 * 
 * This is the function you'll use most often to control your motor.
 * It takes a simple command value and handles all the low-level PWM logic.
 * 
 * @param cmd: Motor command from -1.0 to +1.0
 *             +1.0 = full speed forward
 *              0.0 = stopped
 *             -1.0 = full speed reverse
 */
void BTS7960Motor::setCmd(float cmd) {
  // === INPUT VALIDATION ===
  // Clamp command to safe range (prevent runaway motors!)
  if (cmd > 1.0f) {
cmd = 1.0f;
	}
  else if (cmd < -1.0f) {
cmd = -1.0f;
}
  
  // === AUTO-ENABLE FOR CONVENIENCE ===
  // If motor is disabled but we're trying to move it, enable it automatically
  if (!_enabled) {
enable(true);
}
  
  // === CONVERT COMMAND TO PWM DUTY CYCLE ===
  // Take the absolute value and scale it to our PWM range
  // Example: cmd=0.5, range=1000 → duty=500 (50% PWM)
  uint16_t duty = (uint16_t)(fabsf(cmd) * (float)_range);
  const double minCMD = 0.001f;

  // === DETERMINE DIRECTION AND SET PWM ===
  if (cmd > minCMD) {
    // FORWARD DIRECTION
    // Use right channel (RPWM) for forward movement
    // Make sure left channel (LPWM) is off to prevent conflicts
    writePWM(_rpwm, duty);  // Forward PWM = motor speed
    writePWM(_lpwm, 0);     // Reverse PWM = off
    
  } else if (cmd < -minCMD) {
    // REVERSE DIRECTION  
    // Use left channel (LPWM) for reverse movement
    // Make sure right channel (RPWM) is off to prevent conflicts
    writePWM(_rpwm, 0);     // Forward PWM = off
    writePWM(_lpwm, duty);  // Reverse PWM = motor speed
    
  } else {
    // STOP (COAST MODE)
    // Both PWM channels off - motor will coast to a stop
    // Note: This is "coast" not "brake" - motor isn't actively stopped
    writePWM(_rpwm, 0);
    writePWM(_lpwm, 0);
  }
}

/**
 * Stop the motor immediately (coast mode)
 * 
 * This turns off both PWM channels, allowing the motor to coast to a stop.
 * The motor will gradually slow down due to friction and load.
 */
void BTS7960Motor::stop() {
  // Turn off both PWM channels
  writePWM(_rpwm, 0);
  writePWM(_lpwm, 0);

  digitalWrite(_ren, LOW);
  digitalWrite(_len, LOW);
  
  // Note: Motor driver is still enabled, just not driving the motor
}

/**
 * Completely disable the motor driver
 * 
 * This stops the motor and disables the driver channels.
 * Use this for emergency stops or when you're done using the motor.
 */
void BTS7960Motor::disable() {
  // First stop any movement
  stop();
  
  // Then disable the driver channels
  enable(false);
}

/**
 * Attempt to brake the motor (EXPERIMENTAL - USE WITH CAUTION!)
 * 
 * WARNING: This function's behavior depends on your specific BTS7960 breakout board!
 * 
 * Some boards will actively brake the motor when both channels are driven high.
 * Others may not support this, or it could cause damage.
 * 
 * ALWAYS test this function carefully with your specific hardware setup!
 * Monitor motor current and temperature when testing.
 */
void BTS7960Motor::brake() {
  // Set both PWM channels to maximum
  // This may create a braking effect on some BTS7960 boards
  writePWM(_rpwm, _range);
  writePWM(_lpwm, _range);
  
  // TODO: Consider adding a timeout to prevent overheating
  // TODO: Add current monitoring if available
}

/* 
 * ============================================================================
 * USAGE EXAMPLE:
 * ============================================================================
 * 
 * // Create motor object (pin numbers depend on your wiring)
 * BTS7960Motor myMotor(2, 3, 4, 5);  // rpwm, lpwm, ren, len
 * 
 * void setup() {
 *   // Initialize with 1kHz PWM and 1000-step resolution
 *   myMotor.begin(1000, 1000);
 * }
 * 
 * void loop() {
 *   myMotor.setCmd(0.5);   // Half speed forward
 *   delay(2000);
 *   
 *   myMotor.setCmd(-0.3);  // 30% speed reverse  
 *   delay(2000);
 *   
 *   myMotor.stop();        // Coast to stop
 *   delay(1000);
 * }
 * 
 * ============================================================================
 */




void BTS7960Motor::setCmd(float cmd) {
  // Clean up the input command
  float safeCommand = clampToSafeRange(cmd);
  
  // Make sure motor is ready to move
  ensureMotorIsEnabled();
  
  // Calculate how much power we need
  int powerLevel = calculatePowerLevel(safeCommand);
  
  // Actually move the motor
  moveMotorInDirection(safeCommand, powerLevel);
}

private:
float BTS7960Motor::clampToSafeRange(float command) {
  if (command > 1.0f){

   return 1.0f;
  }
  if (command < -1.0f){
    return -1.0f;
  }
  return command;
}

void BTS7960Motor::ensureMotorIsEnabled() {
  if (!_enabled) {
    enable(true);
  }
}

int BTS7960Motor::calculatePowerLevel(float command) {
  float speedMagnitude = fabsf(command);
  return (int)(speedMagnitude * _range);
}

void BTS7960Motor::moveMotorInDirection(float command, int power) {
  const float deadZone = 0.001f;
  
  if (command > deadZone) {
    goForward(power);
  } else if (command < -deadZone) {
    goReverse(power);
  } else {
    stopMotor();
  }
}

void BTS7960Motor::goForward(int power) {
  writePWM(_rpwm, power);  // Forward channel ON
  writePWM(_lpwm, 0);     // Reverse channel OFF
}

void BTS7960Motor::goReverse(int power) {
  writePWM(_rpwm, 0);     // Forward channel OFF
  writePWM(_lpwm, power); // Reverse channel ON
}

void BTS7960Motor::stopMotor() {
  writePWM(_rpwm, 0);
  writePWM(_lpwm, 0);
}



