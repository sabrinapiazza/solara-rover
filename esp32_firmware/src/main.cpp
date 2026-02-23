#include <Arduino.h>
#include <ArduinoJson.h>
#include <DHT.h>

// ---- Adjust these pins to match your board ----
static const int SOIL_ADC_PIN = 34;   // Use ADC1 (32–39) for WiFi 
static const int DHTPIN = 15;         // Common default, may differ
#define DHTTYPE DHT11                // Try DHT11 first; if nonsense, switch to DHT22

DHT dht(DHTPIN, DHTTYPE);

uint32_t lastSendMs = 0;

int readAdcAveraged(int pin, int samples = 10) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return (int)(sum / samples);
}

// Moisture calibration placeholders (you MUST calibrate)
int moisturePercentFromRaw(int raw) {
  const int RAW_DRY = 3200; // update after you observe readings
  const int RAW_WET = 1400; // update after you observe readings

  int clipped = raw;
  if (clipped > RAW_DRY) clipped = RAW_DRY;
  if (clipped < RAW_WET) clipped = RAW_WET;

  float pct = 100.0f * (RAW_DRY - clipped) / (RAW_DRY - RAW_WET);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)(pct + 0.5f);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12); // 0..4095

  dht.begin();

  Serial.println("{\"status\":\"boot\",\"msg\":\"esp32 soil+temp+humidity ready\"}");
}

void loop() {
  const uint32_t now = millis();

  if (now - lastSendMs >= 1000) {
    lastSendMs = now;

    int moistRaw = readAdcAveraged(SOIL_ADC_PIN);
    int moistPct = moisturePercentFromRaw(moistRaw);

    float hum = dht.readHumidity();
    float tempC = dht.readTemperature();

    StaticJsonDocument<256> doc;
    doc["ts_ms"] = now;
    doc["device"] = "esp32_soil_module";

    doc["soil_raw"] = moistRaw;
    doc["soil_pct"] = moistPct;

    // If DHT read fails, it returns NaN
    if (!isnan(tempC)) doc["air_temp_c"] = tempC;
    if (!isnan(hum))   doc["air_humidity_pct"] = hum;

    serializeJson(doc, Serial);
    Serial.println();
  }
}