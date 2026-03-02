#include <Arduino.h>

#define PULSE_TIMEOUT 150000L   // 150ms max wait
#define DEFAULT_DELAY 10        // 10 ms between measurements
#define DEFAULT_PINGS 5         // number of samples

class SR04 {
public:
    SR04(int echoPin, int triggerPin);

    long Distance();
    long DistanceAvg(int wait = DEFAULT_DELAY, int count = DEFAULT_PINGS);
    void Ping();
    long getDistance();

private:
    long MicrosecondsToCentimeter(long duration);

    int _echoPin, _triggerPin;
    long _duration, _distance;
};

// Constructor
SR04::SR04(int echoPin, int triggerPin) {
    _echoPin = echoPin;
    _triggerPin = triggerPin;

    pinMode(_echoPin, INPUT);
    pinMode(_triggerPin, OUTPUT);

    digitalWrite(_triggerPin, LOW);
    _distance = 999;
}

// Single measurement
long SR04::Distance() {
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(2);

    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(_triggerPin, LOW);

    _duration = pulseIn(_echoPin, HIGH, PULSE_TIMEOUT);

    _distance = MicrosecondsToCentimeter(_duration);

    delay(25);
    return _distance;
}

// Convert time to distance
long SR04::MicrosecondsToCentimeter(long duration) {
    return duration / 58;
}

// Take multiple readings and average (remove min/max)
long SR04::DistanceAvg(int wait, int count) {
    if (count < 3) return Distance();

    long values[count];

    for (int i = 0; i < count; i++) {
        values[i] = Distance();
        delay(wait);
    }

    long minVal = values[0];
    long maxVal = values[0];
    long sum = 0;

    for (int i = 0; i < count; i++) {
        if (values[i] < minVal) minVal = values[i];
        if (values[i] > maxVal) maxVal = values[i];
        sum += values[i];
    }

    sum -= minVal;
    sum -= maxVal;

    return sum / (count - 2);
}

// Simple ping wrapper
void SR04::Ping() {
    Distance();
}

long SR04::getDistance() {
    return _distance;
}


#define TRIG_PIN 18
#define ECHO_PIN 17   // change if needed

SR04 sensor(ECHO_PIN, TRIG_PIN);

// ---------- ARDUINO REQUIRED FUNCTIONS ----------

void setup() {
    Serial.begin(115200);
}

void loop() {
    long distance = sensor.Distance();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    delay(500);
}