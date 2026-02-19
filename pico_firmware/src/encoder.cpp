#include "encoder.h"

/**
 * Encoder Implementation for Raspberry Pi Pico W
 * 
 * This implementation uses hardware interrupts to track encoder pulses
 * in real-time without missing any steps. Perfect for robotics applications
 * that need precise position and speed feedback.
 * 
 * Quadrature Encoding Explanation:
 * ================================
 * Two channels (A & B) create a pattern like this when rotating:
 * 
 * Forward Rotation:    Reverse Rotation:
 * A: ┌─┐ ┌─┐ ┌─┐      A: ┌─┐ ┌─┐ ┌─┐
 *    └─┘ └─┘ └─┘         └─┘ └─┘ └─┘
 * B:  ┌─┐ ┌─┐ ┌─┐      B:┌─┐ ┌─┐ ┌─┐  
 *     └─┘ └─┘ └─┘        └─┘ └─┘ └─┘
 *    A leads B           B leads A
 * 
 * By watching which channel changes first, we can determine direction.
 */

// Static instance pointer for interrupt handling
Encoder* Encoder::_instance = nullptr;

/**
 * Constructor: Set up encoder parameters
 * 
 * @param pinA: GPIO pin for encoder channel A (default: 16 for your setup)
 * @param pinB: GPIO pin for encoder channel B (default: 17 for your setup)  
 * @param pulsesPerRev: How many pulses per motor shaft revolution
 * @param wheelDiameter: Diameter of your rover wheel in meters
 */
Encoder::Encoder(uint8_t pinA, uint8_t pinB, int pulsesPerRev, float wheelDiameter) {
  // Store hardware configuration
  _pinA = pinA;                    // Your encoder channel A on pin 16
  _pinB = pinB;                    // Your encoder channel B on pin 17
  _pulsesPerRev = pulsesPerRev;    // Encoder resolution (typically 64 for 37mm motors)
  _wheelDiameter = wheelDiameter;  // Wheel size for distance calculations
  
  // Initialize tracking variables
  _position = 0;                   // Start at zero position
  _direction = 0;                  // No movement initially
  _lastState = 0;                  // Previous encoder state
  
  // Initialize speed calculation variables
  _lastPosition = 0;               // Position at last speed update
  _lastTime = 0;                   // Time of last speed update
  _currentSpeed = 0;               // Current speed in RPM
  
  // Set up static instance for interrupt handling
  _instance = this;
}

/**
 * Initialize encoder hardware and set up interrupts
 * 
 * This configures the Pico pins and enables interrupts for real-time
 * encoder tracking. Call this once in your setup() function.
 */
void Encoder::begin() {
  // === CONFIGURE GPIO PINS ===
  pinMode(_pinA, INPUT_PULLUP);    // Channel A with internal pullup
  pinMode(_pinB, INPUT_PULLUP);    // Channel B with internal pullup
  
  // Read initial state of both channels
  uint8_t stateA = digitalRead(_pinA);
  uint8_t stateB = digitalRead(_pinB);
  _lastState = (stateA << 1) | stateB;  // Combine into 2-bit state
  
  // === SET UP INTERRUPTS ===
  // Attach interrupts to both channels for maximum resolution
  // CHANGE = trigger on both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(_pinA), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinB), handleEncoderB, CHANGE);
  
  // Initialize timing for speed calculations
  _lastTime = millis();
  
  Serial.print("Encoder initialized on pins ");
  Serial.print(_pinA);
  Serial.print(" and ");
  Serial.print(_pinB);
  Serial.print(" with ");
  Serial.print(_pulsesPerRev);
  Serial.println(" PPR");
}

/**
 * Static interrupt handler for channel A
 * 
 * This function is called automatically whenever pin A changes state.
 * It must be static to work with Arduino's interrupt system.
 */
void Encoder::handleEncoderA() {
  if (_instance != nullptr) {
    _instance->processEncoder();
  }
}

/**
 * Static interrupt handler for channel B
 * 
 * This function is called automatically whenever pin B changes state.
 * It must be static to work with Arduino's interrupt system.
 */
void Encoder::handleEncoderB() {
  if (_instance != nullptr) {
    _instance->processEncoder();
  }
}

/**
 * Process encoder state changes and update position
 * 
 * This is the heart of quadrature decoding. It reads both channels,
 * compares with the previous state, and determines direction and movement.
 */
void Encoder::processEncoder() {
  // Read current state of both encoder channels
  uint8_t stateA = digitalRead(_pinA);
  uint8_t stateB = digitalRead(_pinB);
  uint8_t currentState = (stateA << 1) | stateB;  // Combine into 2-bit value
  
  // Quadrature decoding lookup table
  // This table tells us direction based on state transitions
  static int8_t encoderTable[] = {
     0, -1,  1,  0,   // Previous state 00: 00→0, 01→-1, 10→+1, 11→0
     1,  0,  0, -1,   // Previous state 01: 00→+1, 01→0,  10→0,  11→-1  
    -1,  0,  0,  1,   // Previous state 10: 00→-1, 01→0,  10→0,  11→+1
     0,  1, -1,  0    // Previous state 11: 00→0,  01→+1, 10→-1, 11→0
  };
  
  // Look up direction change based on previous and current state
  int8_t change = encoderTable[(_lastState << 2) | currentState];
  
  // Update position and direction
  _position += change;
  if (change != 0) {
    _direction = change;  // Remember last direction of movement
  }
  
  // Store current state for next comparison
  _lastState = currentState;
}

/**
 * Get current position in encoder pulses
 * 
 * @return: Signed count of encoder pulses
 *          Positive = forward rotation
 *          Negative = reverse rotation
 */
long Encoder::getPosition() {
  // Temporarily disable interrupts to get consistent reading
  noInterrupts();
  long pos = _position;
  interrupts();
  return pos;
}

/**
 * Get distance traveled in meters
 * 
 * This converts encoder pulses to real-world distance using
 * wheel diameter and encoder resolution.
 * 
 * @return: Distance in meters (+ forward, - reverse)
 */
float Encoder::getDistanceMeters() {
  long pulses = getPosition();
  float rotations = pulsesToRotations(pulses);
  return rotationsToMeters(rotations);
}

/**
 * Get current motor speed in RPM
 * 
 * This calculates speed based on position changes over time.
 * Call update() regularly for accurate speed measurements.
 * 
 * @return: Speed in revolutions per minute
 */
float Encoder::getSpeedRPM() {
  return _currentSpeed;
}

/**
 * Get current linear velocity in meters per second
 * 
 * This converts rotational speed (RPM) to linear velocity
 * using the wheel diameter.
 * 
 * @return: Linear velocity in meters per second
 */
float Encoder::getSpeedMPS() {
  // Convert RPM to rotations per second, then to meters per second
  float rotationsPerSecond = _currentSpeed / 60.0;
  float wheelCircumference = M_PI * _wheelDiameter;
  return rotationsPerSecond * wheelCircumference;
}

/**
 * Reset position counter to zero
 * 
 * Useful for setting a new reference point or starting fresh.
 * Speed calculations are not affected.
 */
void Encoder::resetPosition() {
  noInterrupts();
  _position = 0;
  interrupts();
  Serial.println("Encoder position reset to zero");
}

/**
 * Get direction of last movement
 * 
 * @return: 1 = last movement was forward
 *         -1 = last movement was reverse  
 *          0 = no movement detected since last check
 */
int Encoder::getDirection() {
  return _direction;
}

/**
 * Update speed calculations
 * 
 * Call this function regularly (e.g., every 100ms) in your main loop
 * to get accurate speed measurements. The function calculates speed
 * based on position changes since the last update.
 */
void Encoder::update() {
  unsigned long currentTime = millis();
  long currentPosition = getPosition();
  
  // Calculate time elapsed since last update
  unsigned long deltaTime = currentTime - _lastTime;
  
  // Only update if enough time has passed (avoid division by zero)
  if (deltaTime >= 50) {  // Update every 50ms minimum
    
    // Calculate position change
    long deltaPosition = currentPosition - _lastPosition;
    
    // Convert to RPM
    // deltaPosition = pulses moved
    // deltaTime = milliseconds elapsed
    float pulsesPerMinute = (deltaPosition * 60000.0) / deltaTime;
    _currentSpeed = pulsesPerMinute / _pulsesPerRev;
    
    // Store values for next calculation
    _lastPosition = currentPosition;
    _lastTime = currentTime;
  }
}

/**
 * Set wheel diameter for distance calculations
 * 
 * @param diameter: New wheel diameter in meters
 */
void Encoder::setWheelDiameter(float diameter) {
  _wheelDiameter = diameter;
  Serial.print("Wheel diameter set to: ");
  Serial.print(diameter * 1000);  // Convert to mm for display
  Serial.println(" mm");
}

/**
 * Helper function: Convert encoder pulses to wheel rotations
 * 
 * @param pulses: Number of encoder pulses
 * @return: Number of complete wheel rotations
 */
float Encoder::pulsesToRotations(long pulses) {
  return (float)pulses / (float)_pulsesPerRev;
}

/**
 * Helper function: Convert wheel rotations to distance in meters
 * 
 * @param rotations: Number of wheel rotations
 * @return: Distance traveled in meters
 */
float Encoder::rotationsToMeters(float rotations) {
  // Distance = rotations × wheel circumference
  float wheelCircumference = M_PI * _wheelDiameter;
  return rotations * wheelCircumference;
}

/* 
 * ============================================================================
 * USAGE EXAMPLE FOR SOLARA ROVER:
 * ============================================================================
 * 
 * #include "encoder.h"
 * #include "motor_driver.h"
 * 
 * // Create motor and encoder objects
 * BTS7960Motor leftMotor(2, 3, 4, 5);     // Motor driver pins
 * Encoder leftEncoder(16, 17, 64);        // Your pins: 16, 17
 * 
 * void setup() {
 *   Serial.begin(115200);
 *   
 *   // Initialize both motor and encoder
 *   leftMotor.begin(2000, 1000);   // 2kHz PWM, 1000 resolution
 *   leftEncoder.begin();           // Set up interrupts on pins 16 & 17
 * }
 * 
 * void loop() {
 *   // Move forward
 *   leftMotor.setCmd(0.5);         // 50% speed forward
 *   
 *   // Update encoder calculations
 *   leftEncoder.update();
 *   
 *   // Read feedback
 *   float distance = leftEncoder.getDistanceMeters();
 *   float speed = leftEncoder.getSpeedRPM();
 *   int direction = leftEncoder.getDirection();
 *   
 *   // Print status
 *   Serial.print("Distance: ");
 *   Serial.print(distance, 3);
 *   Serial.print("m, Speed: ");
 *   Serial.print(speed, 1);
 *   Serial.print(" RPM, Direction: ");
 *   Serial.println(direction > 0 ? "Forward" : direction < 0 ? "Reverse" : "Stopped");
 *   
 *   delay(100);  // 100ms update rate
 * }
 * 
 * ============================================================================
 */