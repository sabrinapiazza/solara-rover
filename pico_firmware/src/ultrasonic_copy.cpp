#include "pico/stdlib.h"

#define PULSE_TIMEOUT 150000L   // 150ms max time to wait for echo pulse
#define DEFAULT_DELAY 10 // 10 ms between measurements
#define DEFAULT_PINGS 5 // takes 5 readings, remove low high average rest


class SR04 {
public:
    /**
 * Constructor for the SR04 ultrasonic sensor.
 *
 * The sensor has 4 pins: VCC, GND, TRIG, and ECHO.
 *
 * echoPin    -> GPIO pin connected to the ECHO pin (input)
 * triggerPin -> GPIO pin connected to the TRIG pin (output)
 *
 * The trigger pin sends a 10µs HIGH pulse to start a measurement.
 * The echo pin receives a pulse whose duration represents distance.
 */
    SR04(int echoPin, int triggerPin);

    /**
    * Do a measurement for this sensor. Return distance as long
    * in centimenter
    * \return long distance in centimeter
    */
    long Distance();
   
   /**
 * Takes multiple distance measurements and returns an averaged result.
 *
 * To reduce noise and incorrect spikes, the smallest and largest
 * readings are removed before calculating the average.
 *
 * @param wait  Delay in milliseconds between measurements (default: DEFAULT_DELAY)
 * @param count Number of measurements to take (default: DEFAULT_PINGS)
 * @return Distance in centimeters
 */

    long DistanceAvg(int wait=DEFAULT_DELAY, int count=DEFAULT_PINGS);
   
    /**
    * Do only a ping. Get result with method getDistance()
    *
    * \param keine
    */
    void Ping() ;
   
   /**
 * Returns the most recent distance measurement in centimeters.
 *
 * Note: Ping() must be called first to update the measurement.
 *
 * @return Distance in centimeters
 */
    long getDistance();
   
private:
   /**
 * Converts the echo pulse duration (in microseconds)
 * into distance in centimeters.
 *
 * The sensor measures the time it takes for a sound wave
 * to travel to an object and back.
 *
 * Since sound travels at ~340 m/s and the signal travels
 * both to the object and back, the distance can be calculated as:
 *
 * distance (cm) = duration / 58
 */
    long MicrosecondsToCentimeter(long duration);
   
   
    int _echoPin, _triggerPin;
    long _duration, _distance;
    long measurePulse();
   
};

//implementations
SR04::SR04(int echoPin, int triggerPin) {
    _echoPin = echoPin;
    _triggerPin = triggerPin;
    gpio_init(_echoPin);
    gpio_set_dir(_echoPin, GPIO_IN);
    gpio_init(_triggerPin);
    gpio_set_dir(_triggerPin, GPIO_OUT);


    _distance = 999;
}

long SR04::Distance() {
    long d = 0;
    _duration = 0;
    gpio_put(_triggerPin, 0);
    sleep_us(2);
    gpio_put(_triggerPin, 1);
    sleep_us(10);
    gpio_put(_triggerPin, 0);
    sleep_us(2);


    _duration = measurePulse();
    d = MicrosecondsToCentimeter(_duration);
sleep_ms(25);
    return d;
}

long SR04::MicrosecondsToCentimeter(long duration) {
    return duration / 58; //distance = (speed of sound × time) / 2
}

long SR04::measurePulse() {
    absolute_time_t timeout = make_timeout_time_us(PULSE_TIMEOUT);

    // Wait for echo HIGH
    while (gpio_get(_echoPin) == 0) {
        if (time_reached(timeout)){
   		 return 0;
}
   }

    absolute_time_t start = get_absolute_time();

    // Wait for echo LOW
    while (gpio_get(_echoPin) == 1) {
if (time_reached(timeout)){
  		return 0;
}
    }

    return absolute_time_diff_us(start, get_absolute_time());
}