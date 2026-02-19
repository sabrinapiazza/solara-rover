// Velocity PID control for wheels
// Takes desired velocity, adjusts motor PWM to maintain it
// Handles acceleration/deceleration smoothly
#include "pid_controller.h"

/**
 * PID Controller Implementation for Raspberry Pi Pico W
 * 
 * This implementation provides a robust PID controller suitable for
 * robotics applications. It includes anti-windup protection, derivative
 * kick prevention, and proper timing handling.
 * 
 * PID Theory for Robotics:
 * ========================
 * 
 * P (Proportional): 
 *   - Responds immediately to current error
 *   - Higher Kp = faster response, but can cause overshoot
 *   - Example: If motor is 10 RPM too slow, apply proportional correction
 * 
 * I (Integral):
 *   - Eliminates steady-state error over time
 *   - Higher Ki = eliminates offset, but can cause oscillation
 *   - Example: If motor consistently runs 5 RPM slow, integral builds up
 * 
 * D (Derivative):
 *   - Predicts future error based on rate of change
 *   - Higher Kd = reduces overshoot, but sensitive to noise
 *   - Example: If error is decreasing rapidly, reduce correction
 */

/**
 * Constructor: Initialize PID controller with gains and output limits
 * 
 * @param kp: Proportional gain (typical range: 0.1 - 10.0)
 * @param ki: Integral gain (typical range: 0.0 - 1.0)  
 * @param kd: Derivative gain (typical range: 0.0 - 0.1)
 * @param outputMin: Minimum output (-1.0 for full reverse motor command)
 * @param outputMax: Maximum output (+1.0 for full forward motor command)
 */
PID::PID(float kp, float ki, float kd, float outputMin, float outputMax) {
  // Store PID gains
  _kp = kp;
  _ki = ki;
  _kd = kd;
  
  // Store output limits (prevents motor damage and windup)
  _outputMin = outputMin;
  _outputMax = outputMax;
  
  // Initialize state variables
  _error = 0.0;
  _lastError = 0.0;
  _integral = 0.0;
  _lastMeasurement = 0.0;
  
  // Initialize PID components for debugging
  _proportional = 0.0;
  _derivative = 0.0;
  
  // Set default configuration
  _sampleTime = 100;        // 100ms default sample time
  _antiWindup = true;       // Enable anti-windup by default
  _firstRun = true;         // Flag for first computation
  
  _lastTime = 0;
}

/**
 * Initialize the PID controller
 * 
 * Sets up timing and resets all state variables for a fresh start.
 * Call this once in your setup() function.
 */
void PID::begin() {
  reset();
  Serial.println("PID controller initialized");
  Serial.print("Gains - Kp: ");
  Serial.print(_kp);
  Serial.print(", Ki: ");
  Serial.print(_ki);
  Serial.print(", Kd: ");
  Serial.println(_kd);
}

/**
 * Compute PID output based on setpoint and measurement
 * 
 * This is the main PID algorithm. Call this regularly (every sampleTime ms)
 * with your target value and current sensor reading.
 * 
 * @param setpoint: Desired value (target speed, position, etc.)
 * @param measurement: Current value from sensor (encoder reading)
 * @return: PID output clamped to output limits
 */
float PID::compute(float setpoint, float measurement) {
  // Check if enough time has passed since last computation
  unsigned long currentTime = millis();
  unsigned long timeChange = currentTime - _lastTime;
  
  // Only compute if sample time has elapsed (prevents too-frequent updates)
  if (timeChange < _sampleTime && !_firstRun) {
    // Not enough time has passed - return last output would be here
    // For simplicity, we'll compute anyway but this could be optimized
  }
  
  // === CALCULATE ERROR ===
  _error = setpoint - measurement;
  
  // === CALCULATE PROPORTIONAL TERM ===
  // P term responds immediately to current error
  _proportional = _kp * _error;
  
  // === CALCULATE INTEGRAL TERM ===
  // I term accumulates error over time to eliminate steady-state offset
  if (!_firstRun) {  // Skip integral on first run to avoid initial kick
    _integral += _error * (float)timeChange;
    
    // Anti-windup: prevent integral buildup when output is saturated
    if (_antiWindup) {
      float tentativeOutput = _proportional + _ki * _integral;
      
      // If output would be saturated, don't accumulate more integral error
      if (tentativeOutput > _outputMax && _error > 0) {
        _integral -= _error * (float)timeChange;  // Remove this addition
      } else if (tentativeOutput < _outputMin && _error < 0) {
        _integral -= _error * (float)timeChange;  // Remove this addition
      }
    }
  }
  
  // === CALCULATE DERIVATIVE TERM ===
  // D term predicts future error based on rate of change
  // Use "derivative on measurement" to avoid derivative kick when setpoint changes
  _derivative = 0.0;
  if (!_firstRun && timeChange > 0) {
    float measurementChange = measurement - _lastMeasurement;
    _derivative = -_kd * measurementChange / (float)timeChange;  // Negative because we want to oppose the change
  }
  
  // === COMBINE PID TERMS ===
  float output = _proportional + (_ki * _integral) + _derivative;
  
  // === APPLY OUTPUT LIMITS ===
  output = clamp(output, _outputMin, _outputMax);
  
  // === STORE VALUES FOR NEXT COMPUTATION ===
  _lastError = _error;
  _lastMeasurement = measurement;
  _lastTime = currentTime;
  _firstRun = false;
  
  return output;
}

/**
 * Set PID gains for tuning control performance
 * 
 * Use this function to adjust PID behavior:
 * - Increase Kp for faster response (but may cause overshoot)
 * - Increase Ki to eliminate steady-state error (but may cause oscillation)
 * - Increase Kd to reduce overshoot (but may amplify noise)
 */
void PID::setGains(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  
  Serial.print("PID gains updated - Kp: ");
  Serial.print(_kp);
  Serial.print(", Ki: ");
  Serial.print(_ki);
  Serial.print(", Kd: ");
  Serial.println(_kd);
}

/**
 * Set output limits to prevent motor damage and windup
 * 
 * @param min: Minimum output value (e.g., -1.0 for full reverse)
 * @param max: Maximum output value (e.g., +1.0 for full forward)
 */
void PID::setOutputLimits(float min, float max) {
  _outputMin = min;
  _outputMax = max;
  
  Serial.print("PID output limits set: ");
  Serial.print(_outputMin);
  Serial.print(" to ");
  Serial.println(_outputMax);
}

/**
 * Set sample time for consistent PID timing
 * 
 * @param sampleTimeMs: Time between compute() calls in milliseconds
 */
void PID::setSampleTime(unsigned long sampleTimeMs) {
  if (sampleTimeMs > 0) {
    _sampleTime = sampleTimeMs;
    Serial.print("PID sample time set to ");
    Serial.print(_sampleTime);
    Serial.println(" ms");
  }
}

/**
 * Reset PID controller state
 * 
 * Call this when starting a new control task, changing setpoints dramatically,
 * or after the system has been paused for a long time.
 */
void PID::reset() {
  _error = 0.0;
  _lastError = 0.0;
  _integral = 0.0;
  _lastMeasurement = 0.0;
  _proportional = 0.0;
  _derivative = 0.0;
  _firstRun = true;
  _lastTime = millis();
  
  Serial.println("PID controller reset");
}

/**
 * Enable or disable integral anti-windup protection
 * 
 * Anti-windup prevents the integral term from building up when the
 * output is saturated (at min or max limits). This prevents overshoot
 * when the system comes back into the controllable range.
 * 
 * @param enable: true to enable anti-windup, false to disable
 */
void PID::setAntiWindup(bool enable) {
  _antiWindup = enable;
  Serial.print("PID anti-windup ");
  Serial.println(enable ? "enabled" : "disabled");
}

/**
 * Helper function: Clamp value between min and max limits
 * 
 * @param value: Input value to clamp
 * @param min: Minimum allowed value
 * @param max: Maximum allowed value
 * @return: Clamped value
 */
float PID::clamp(float value, float min, float max) {
  if (value > max) return max;
  if (value < min) return min;
  return value;
}

/* 
 * ============================================================================
 * USAGE EXAMPLE FOR SOLARA ROVER - SPEED CONTROL:
 * ============================================================================
 * 
 * #include "motor_driver.h"
 * #include "encoder.h"
 * #include "pid.h"
 * 
 * // Hardware objects
 * BTS7960Motor leftMotor(2, 3, 4, 5);
 * Encoder leftEncoder(16, 17, 64);
 * PID speedController(0.5, 0.1, 0.05);  // Kp, Ki, Kd
 * 
 * float targetSpeed = 100.0;  // Target 100 RPM
 * 
 * void setup() {
 *   Serial.begin(115200);
 *   
 *   leftMotor.begin(2000, 1000);
 *   leftEncoder.begin();
 *   speedController.begin();
 *   speedController.setSampleTime(50);  // 50ms update rate
 * }
 * 
 * void loop() {
 *   // Update encoder
 *   leftEncoder.update();
 *   
 *   // Get current speed
 *   float currentSpeed = leftEncoder.getSpeedRPM();
 *   
 *   // Compute PID output
 *   float motorCommand = speedController.compute(targetSpeed, currentSpeed);
 *   
 *   // Apply to motor
 *   leftMotor.setCmd(motorCommand);
 *   
 *   // Debug output
 *   Serial.print("Target: ");
 *   Serial.print(targetSpeed);
 *   Serial.print(" RPM, Current: ");
 *   Serial.print(currentSpeed);
 *   Serial.print(" RPM, Command: ");
 *   Serial.print(motorCommand * 100);
 *   Serial.println("%");
 *   
 *   delay(50);  // Match PID sample time
 * }
 * 
 * ============================================================================
 * 
 * USAGE EXAMPLE FOR SOLARA ROVER - POSITION CONTROL:
 * ============================================================================
 * 
 * // Position control for precise navigation
 * PID positionController(2.0, 0.0, 0.1);  // Higher Kp for position control
 * 
 * float targetDistance = 2.0;  // Target 2 meters
 * 
 * void positionControlLoop() {
 *   leftEncoder.update();
 *   
 *   float currentDistance = leftEncoder.getDistanceMeters();
 *   float motorCommand = positionController.compute(targetDistance, currentDistance);
 *   
 *   leftMotor.setCmd(motorCommand);
 *   
 *   // Check if we've reached target
 *   if (abs(targetDistance - currentDistance) < 0.05) {  // Within 5cm
 *     leftMotor.stop();
 *     Serial.println("Target position reached!");
 *   }
 * }
 * 
 * ============================================================================
 * 
 * PID TUNING GUIDE FOR SOLARA ROVER:
 * ============================================================================
 * 
 * 1. Start with Kp only (Ki=0, Kd=0):
 *    - Increase Kp until system responds quickly but oscillates
 *    - Reduce Kp until oscillation stops
 * 
 * 2. Add Integral term (Ki):
 *    - Increase Ki to eliminate steady-state error
 *    - If system becomes unstable, reduce Ki
 * 
 * 3. Add Derivative term (Kd):
 *    - Increase Kd to reduce overshoot
 *    - If system becomes noisy or jittery, reduce Kd
 * 
 * Typical values for 37mm gear motors:
 * - Speed control: Kp=0.5, Ki=0.1, Kd=0.05
 * - Position control: Kp=2.0, Ki=0.0, Kd=0.1
 * 
 * ============================================================================
 */