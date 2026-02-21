#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/**
 * PID Controller Class for Raspberry Pi Pico W
 * 
 * This class implements a Proportional-Integral-Derivative (PID) controller
 * for precise motor speed and position control. Perfect for maintaining
 * constant speeds, following trajectories, and accurate positioning.
 * 
 * How PID Control Works:
 * - P (Proportional): Responds to current error
 * - I (Integral): Eliminates steady-state error over time  
 * - D (Derivative): Reduces overshoot and oscillations
 * 
 * PID Formula: output = Kp*error + Ki*integral + Kd*derivative
 * 
 * Integration with SOLARA Rover:
 * - Encoder provides current position/speed (feedback)
 * - PID calculates required motor command
 * - Motor driver executes the command
 * - Loop repeats for continuous control
 */
class PID {
public:
  // === PUBLIC INTERFACE ===
  
  /**
   * Constructor: Set up PID controller parameters
   * 
   * @param kp: Proportional gain (how aggressively to respond to current error)
   * @param ki: Integral gain (how aggressively to respond to accumulated error)  
   * @param kd: Derivative gain (how aggressively to respond to error rate of change)
   * @param outputMin: Minimum output value (e.g., -1.0 for full reverse)
   * @param outputMax: Maximum output value (e.g., +1.0 for full forward)
   */
  PID(float kp = 1.0, float ki = 0.0, float kd = 0.0, 
      float outputMin = -1.0, float outputMax = 1.0);
  
  /**
   * Initialize the PID controller
   * Call this once in your setup() function
   */
  void begin();
  
  /**
   * Compute PID output based on setpoint and current measurement
   * 
   * @param setpoint: Desired value (target speed, position, etc.)
   * @param measurement: Current value from sensor (encoder reading)
   * @return: PID output (motor command between outputMin and outputMax)
   */
  float compute(float setpoint, float measurement);
  
  /**
   * Set PID gains for tuning control performance
   * 
   * @param kp: Proportional gain
   * @param ki: Integral gain  
   * @param kd: Derivative gain
   */
  void setGains(float kp, float ki, float kd);
  
  /**
   * Set output limits to prevent windup and motor damage
   * 
   * @param min: Minimum output value
   * @param max: Maximum output value
   */
  void setOutputLimits(float min, float max);
  
  /**
   * Set sample time for consistent timing
   * 
   * @param sampleTimeMs: Time between compute() calls in milliseconds
   */
  void setSampleTime(unsigned long sampleTimeMs);
  
  /**
   * Reset PID controller state
   * Call this when starting new control task or after long pauses
   */
  void reset();
  
  /**
   * Enable or disable integral term anti-windup
   * Prevents integral buildup when output is saturated
   * 
   * @param enable: true to enable anti-windup, false to disable
   */
  void setAntiWindup(bool enable);
  
  /**
   * Get current PID gains (for tuning and debugging)
   */
  float getKp() { return _kp; }
  float getKi() { return _ki; }
  float getKd() { return _kd; }
  
  /**
   * Get individual PID components (for debugging)
   */
  float getProportional() { return _proportional; }
  float getIntegral() { return _integral; }
  float getDerivative() { return _derivative; }
  
  /**
   * Get current error value
   */
  float getError() { return _error; }

private:
  // === PID GAINS ===
  float _kp;                    // Proportional gain
  float _ki;                    // Integral gain  
  float _kd;                    // Derivative gain
  
  // === OUTPUT LIMITS ===
  float _outputMin;             // Minimum output value
  float _outputMax;             // Maximum output value
  
  // === TIMING ===
  unsigned long _sampleTime;    // Sample time in milliseconds
  unsigned long _lastTime;      // Time of last computation
  
  // === PID STATE VARIABLES ===
  float _error;                 // Current error
  float _lastError;             // Previous error (for derivative)
  float _integral;              // Accumulated error (for integral)
  float _lastMeasurement;       // Previous measurement (for derivative kick prevention)
  
  // === PID COMPONENTS (for debugging) ===
  float _proportional;          // P term value
  float _derivative;            // D term value
  
  // === CONFIGURATION ===
  bool _antiWindup;             // Anti-windup enable flag
  bool _firstRun;               // Flag to handle first computation
  
  // === HELPER FUNCTIONS ===
  /**
   * Clamp value between min and max limits
   */
  float clamp(float value, float min, float max);
};