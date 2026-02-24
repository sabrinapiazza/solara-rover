// Controls BTM7960 motor drivers for wheels
// PWM output to drive motors forward/backward
// Speed control functions
// Motor enable/disable

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
 * @param pwmFreqHz: PWM frequency in Hz (recommended: 1000-5000 Hz for optimal balance)
 * @param pwmRange:  PWM resolution (1000 = 0.1% resolution, 255 = Arduino standard)
 */
void BTS7960Motor::begin(uint32_t pwmFreqHz = 1000, uint16_t pwmRange = 1000) {
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
  
  if (on) {
    // Turn on both motor driver channels 
    digitalWrite(_ren, HIGH);  // Enable forward direction channel
    digitalWrite(_len, HIGH);  // Enable reverse direction channel
  } else {
    // Turn off both motor driver channels 
    digitalWrite(_ren, LOW);   // Disable forward direction channel
    digitalWrite(_len, LOW);   // Disable reverse direction channel
  }
  
  // Safety: If disabling, make sure no PWM is being sent
  if (!on) {
    analogWrite(_rpwm, 0);
    analogWrite(_lpwm, 0);
  }
}

/**
 * Helper function: Safely apply power to a motor channel
 * 
 * This prevents accidentally sending PWM values that are too high,
 * which could damage the motor or cause unexpected behavior.
 * 
 * @param whichPin: The pin number to send power to (either _rpwm or _lpwm)
 * @param howMuchPower: How much power to apply (0 = no power, _range = full power)
 */
void BTS7960Motor::applyPowerToChannel(uint8_t whichPin, uint16_t howMuchPower) {
  // Safety check: Don't allow power levels above our maximum
  if (howMuchPower > _range) {
    howMuchPower = _range;  // Cap at maximum safe level
  }
  
  // Send the PWM signal to control motor speed
  analogWrite(whichPin, howMuchPower);
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
void BTS7960Motor::setCmd(double cmd) {
  // Clean up the input command
  double safeCommand = clampToSafeRange(cmd);
  
  // Make sure motor is ready to move
  ensureMotorIsEnabled();
  
  // Calculate how much power we need
  int powerLevel = calculatePowerLevel(safeCommand);
  
  // Actually move the motor
  moveMotorInDirection(safeCommand, powerLevel);
}

/**
 * Helper function: Limit speed to safe range
 * 
 * @param command: Raw command value
 * @return: Command clamped to -1.0 to +1.0 range
 */
double BTS7960Motor::clampToSafeRange(double command) {
  if (command > 1.0f) {
    return 1.0f;  // Maximum forward speed
  }
  if (command < -1.0f) {
    return -1.0f; // Maximum reverse speed
  }
  return command;   // Speed is already safe
}

/**
 * Helper function: Wake up motor if needed
 */
void BTS7960Motor::ensureMotorIsEnabled() {
  if (!_enabled) {
    enable(true);
  }
}

/**
 * Helper function: Convert speed to electrical power level
 * 
 * @param command: Speed command (-1.0 to +1.0)
 * @return: PWM power level (0 to _range)
 */
uint16_t BTS7960Motor::calculatePowerLevel(double command) {
  // Remove the direction (+ or -) and just get magnitude
  const double speedMagnitude = fabsf(command);
  
  // Convert from 0.0-1.0 scale to 0-_range scale
  // Example: 0.5 speed with _range=1000 → 500 power level
  return static_cast<uint16_t>(speedMagnitude * _range);
}

/**
 * Helper function: Drive motor in the correct direction
 * 
 * @param command: Speed command (with direction)
 * @param power: Power level to apply
 */
void BTS7960Motor::moveMotorInDirection(double command, int power) {
  const double minimumMovement = 0.001f;  // Dead zone to prevent tiny movements
  
  if (command > minimumMovement) {
    goForward(power);
  } else if (command < -minimumMovement) {
    goReverse(power);
  } else {
    stopMotor();
  }
}

/**
 * Helper function: Drive motor forward
 */
void BTS7960Motor::goForward(int power) {
  applyPowerToChannel(_rpwm, power);  // Forward channel ON
  applyPowerToChannel(_lpwm, 0);      // Reverse channel OFF
}

/**
 * Helper function: Drive motor in reverse
 */
void BTS7960Motor::goReverse(int power) {
  applyPowerToChannel(_rpwm, 0);      // Forward channel OFF
  applyPowerToChannel(_lpwm, power);  // Reverse channel ON
}

/**
 * Helper function: Stop motor movement
 */
void BTS7960Motor::stopMotor() {
  applyPowerToChannel(_rpwm, 0);  // Both channels off
  applyPowerToChannel(_lpwm, 0);  // Motor coasts to stop
}

/**
 * Stop the motor immediately (coast mode)
 * 
 * This turns off both PWM channels, allowing the motor to coast to a stop.
 * The motor will gradually slow down due to friction and load.
 * Motor driver channels remain enabled and ready.
 */
void BTS7960Motor::stop() {
  // Turn off both PWM channels (motor coasts to stop)
  stopMotor();
  
  // Note: Enable pins stay HIGH - driver is ready for next command
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
  
  // Then disable the driver channels completely
  enable(false);
}

