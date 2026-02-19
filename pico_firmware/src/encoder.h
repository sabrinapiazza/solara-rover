#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

/**
 * Encoder Class for Raspberry Pi Pico W
 * 
 * This class reads rotary encoder signals to track motor position,
 * speed, and direction. Perfect for closed-loop motor control and
 * precise navigation in robotics applications.
 * 
 * How Quadrature Encoding Works:
 * - Two channels (A & B) that are 90° out of phase
 * - Direction determined by which channel leads the other
 * - Position tracked by counting rising/falling edges
 * - High resolution from both channels and both edges
 * 
 * Hardware Setup:
 * - Encoder Channel A → Pico pin 16 (GP16)
 * - Encoder Channel B → Pico pin 17 (GP17)
 * - Encoder VCC → 3.3V or 5V (check motor specs)
 * - Encoder GND → Ground
 */
class Encoder {
public:
  // === PUBLIC INTERFACE ===
  
  /**
   * Constructor: Set up encoder pins and parameters
   * 
   * @param pinA: GPIO pin for encoder channel A (default: 16)
   * @param pinB: GPIO pin for encoder channel B (default: 17)
   * @param pulsesPerRev: Encoder resolution (pulses per motor revolution)
   * @param wheelDiameter: Wheel diameter in meters (for distance calculations)
   */
  Encoder(uint8_t pinA = 16, uint8_t pinB = 17, 
          int pulsesPerRev = 64, float wheelDiameter = 0.065);
  
  /**
   * Initialize the encoder hardware and interrupts
   * Call this once in your setup() function
   */
  void begin();
  
  /**
   * Get current position in encoder pulses
   * @return: Signed integer (+ = forward, - = reverse)
   */
  long getPosition();
  
  /**
   * Get distance traveled in meters
   * @return: Distance in meters (+ = forward, - = reverse)
   */
  float getDistanceMeters();
  
  /**
   * Get current speed in RPM
   * @return: Motor speed in revolutions per minute
   */
  float getSpeedRPM();
  
  /**
   * Get current speed in meters per second
   * @return: Linear velocity in m/s
   */
  float getSpeedMPS();
  
  /**
   * Reset position counter to zero
   * Useful for starting fresh or setting reference points
   */
  void resetPosition();
  
  /**
   * Get direction of last movement
   * @return: 1 = forward, -1 = reverse, 0 = stopped
   */
  int getDirection();
  
  /**
   * Update speed calculations (call regularly in loop)
   * This function calculates speed based on position changes
   */
  void update();
  
  /**
   * Set wheel diameter for distance calculations
   * @param diameter: Wheel diameter in meters
   */
  void setWheelDiameter(float diameter);

private:
  // === HARDWARE CONFIGURATION ===
  uint8_t _pinA;              // Encoder channel A pin
  uint8_t _pinB;              // Encoder channel B pin
  int _pulsesPerRev;          // Encoder resolution
  float _wheelDiameter;       // Wheel diameter in meters
  
  // === POSITION TRACKING ===
  volatile long _position;    // Current position in pulses
  volatile int _direction;    // Current direction (1, -1, or 0)
  
  // === SPEED CALCULATION ===
  long _lastPosition;         // Position at last speed calculation
  unsigned long _lastTime;    // Time of last speed calculation
  float _currentSpeed;        // Current speed in RPM
  
  // === INTERRUPT STATE ===
  volatile uint8_t _lastState;  // Previous A/B state for decoding
  
  // === INTERRUPT HANDLERS ===
  /**
   * Static interrupt handler for encoder A pin
   * Must be static to work with Arduino interrupt system
   */
  static void handleEncoderA();
  
  /**
   * Static interrupt handler for encoder B pin  
   * Must be static to work with Arduino interrupt system
   */
  static void handleEncoderB();
  
  /**
   * Instance-specific interrupt processing
   * Called by static handlers to update position
   */
  void processEncoder();
  
  // === HELPER FUNCTIONS ===
  /**
   * Convert pulses to wheel rotations
   */
  float pulsesToRotations(long pulses);
  
  /**
   * Convert rotations to distance traveled
   */
  float rotationsToMeters(float rotations);
  
  // === STATIC INSTANCE POINTER ===
  // Needed for interrupt handling (Arduino limitation)
  static Encoder* _instance;
};

#endif // ENCODER_H
