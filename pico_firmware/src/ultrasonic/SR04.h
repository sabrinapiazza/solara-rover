#pragma once
#include "pico/stdlib.h"

#define PULSE_TIMEOUT_US 150000L
#define DEFAULT_DELAY_MS 10
#define DEFAULT_PINGS    5

class SR04 {
public:
  SR04(uint echoPin, uint triggerPin);

  long Distance();
  long DistanceAvg(int wait_ms = DEFAULT_DELAY_MS, int count = DEFAULT_PINGS);
  void Ping();
  long getDistance() const;

private:
  long MicrosecondsToCentimeter(long duration_us);
  long measurePulse();

  uint _echoPin, _triggerPin;
  long _duration, _distance;
};