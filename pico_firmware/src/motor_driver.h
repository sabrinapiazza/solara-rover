#pragma once
#include <Arduino.h>

/*
  BTS7960 Motor Driver (1 motor)
  - RPWM / LPWM: PWM pins (speed + direction)
  - R_EN / L_EN: enable pins (digital)
  Command convention:
    cmd in [-1.0, +1.0]
      +cmd = forward (RPWM active)
      -cmd = reverse (LPWM active)
*/

class BTS7960Motor {
public:
  BTS7960Motor(uint8_t rpwm, uint8_t lpwm, uint8_t ren, uint8_t len);

  // Call once in setup()
  void begin(uint32_t pwmFreqHz = 1000, uint16_t pwmRange = 1000);

  // Enable/disable the driver (EN pins)
  void enable(bool on);

// Prevents high PWM that causes damages
  void applyPowerToChannel(uint8_t whichPin, uint16_t howMuchPower);

  // Motor command: [-1.0 .. +1.0]
  void setCmd(double cmd);

// Limits speed to safe range
  double clampToSafeRange(double command);

// Wakes up motor if necessary
  void ensureMotorIsEnabled();

// Converts speed to electrical power level
  uint16_t calculatePowerLevel(double command);

// Drives motor in the correct direction
void moveMotorInDirection(double command, int power);

// Motor goes forward
  void goForward(int power);

// Motor goes in reverse
  void goReverse;

// Stops motor movement
void stopMotor();

  // Convenience
  void stop();     // coast
  void disable();  // disable EN pins + PWM=0

  // Optional (behavior varies by board; use cautiously)
  void brake();

  // Getters (useful for debugging)
  bool isEnabled() const { 
return _enabled; 
  }
  uint16_t range() const { 
return _range; 
  }

private:
  uint8_t _rpwm, _lpwm, _ren, _len;
  uint16_t _range;
  bool _enabled;

  void writePWM(uint8_t pin, uint16_t duty);
};
