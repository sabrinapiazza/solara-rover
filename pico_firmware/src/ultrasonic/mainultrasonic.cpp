#include <stdio.h>
#include "pico/stdlib.h"
#include "SR04.h"

static constexpr uint TRIG_PIN = 19;
static constexpr uint ECHO_PIN = 20;

int main() {
  stdio_init_all();     // enables USB serial if your CMake enables it
  sleep_ms(1500);       // give USB serial time to connect

  printf("SR04 test starting...\n");
  SR04 sensor(ECHO_PIN, TRIG_PIN);

  while (true) {
    long cm = sensor.Distance();
    printf("Distance: %ld cm\n", cm);
    sleep_ms(500);
  }
}