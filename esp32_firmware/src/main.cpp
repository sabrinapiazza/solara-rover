#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 sensor module initialized");
}

void loop() {
  int soil_raw = analogRead(34);  // ADC pin (adjust later)

  Serial.print("Soil raw: ");
  Serial.println(soil_raw);

  delay(2000);
}
