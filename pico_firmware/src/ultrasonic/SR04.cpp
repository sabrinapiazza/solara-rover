#include "SR04.h"

SR04::SR04(uint echoPin, uint triggerPin)
: _echoPin(echoPin), _triggerPin(triggerPin), _duration(0), _distance(999)
{
  gpio_init(_echoPin);
  gpio_set_dir(_echoPin, GPIO_IN);

  gpio_init(_triggerPin);
  gpio_set_dir(_triggerPin, GPIO_OUT);
  gpio_put(_triggerPin, 0);
}

long SR04::Distance() {
  gpio_put(_triggerPin, 0);
  sleep_us(2);
  gpio_put(_triggerPin, 1);
  sleep_us(10);
  gpio_put(_triggerPin, 0);

  _duration = measurePulse();
  _distance = MicrosecondsToCentimeter(_duration);

  sleep_ms(25);
  return _distance;
}

long SR04::MicrosecondsToCentimeter(long duration_us) {
  return duration_us / 58;
}

long SR04::measurePulse() {
  absolute_time_t deadline = make_timeout_time_us(PULSE_TIMEOUT_US);

  while (gpio_get(_echoPin) == 0) {
    if (time_reached(deadline)) return 0;
  }
  absolute_time_t start = get_absolute_time();

  while (gpio_get(_echoPin) == 1) {
    if (time_reached(deadline)) return 0;
  }

  return absolute_time_diff_us(start, get_absolute_time());
}

long SR04::DistanceAvg(int wait_ms, int count) {
  if (count < 3) return Distance();
  if (count > DEFAULT_PINGS) count = DEFAULT_PINGS;

  long values[DEFAULT_PINGS];
  for (int i = 0; i < count; i++) {
    values[i] = Distance();
    sleep_ms(wait_ms);
  }

  long minVal = values[0], maxVal = values[0], sum = 0;
  for (int i = 0; i < count; i++) {
    if (values[i] < minVal) minVal = values[i];
    if (values[i] > maxVal) maxVal = values[i];
    sum += values[i];
  }

  sum -= minVal;
  sum -= maxVal;
  return sum / (count - 2);
}

void SR04::Ping() { Distance(); }
long SR04::getDistance() const { return _distance; }