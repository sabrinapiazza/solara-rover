#include <Arduino.h>
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

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

#endif