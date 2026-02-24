# BME680 sensor driver
# Reads temperature, humidity, pressure, air quality
# Returns environmental measurements
# https://www.rp2040learning.com/code/circuitpython/raspberry-pi-pico-and-bme680-gas-sensor-circuitpython-example.php
# https://docs.circuitpython.org/projects/bme680/en/latest/api.html#implementation-notes
# https://docs.circuitpython.org/projects/bme680/en/latest/

import adafruit_bme680
import time
import board
import busio

def get_data():
    # Single read of all BME680 sensors. Returns a dict for collector.py.
    i2c = busio.I2C(scl=board.GP1, sda=board.GP0)
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, 0x76)
    bme680.sea_level_pressure = 1020.7  # hPa - adjust to local sea-level pressure

    return {
        "temperature_c": round(bme680.temperature, 2),  # degrees Celsius
        "humidity_rh":   round(bme680.relative_humidity, 2),  # RH %
        "pressure_hpa":  round(bme680.pressure, 2),  # hectoPascals
        "gas_ohms":      bme680.gas,  # gas resistance in ohms
        "altitude_m":    round(bme680.altitude, 2),  # meters
    }

def main():
    # Standalone loop for testing environment.py directly.
    print("BME680 initialized. Reading data...")
    try:
        while True:
            data = get_data()
            print(
                f"Temp: {data['temperature_c']} °C, "
                f"Humidity: {data['humidity_rh']} %, "
                f"Pressure: {data['pressure_hpa']} hPa, "
                f"Gas: {data['gas_ohms']} ohms, "
                f"Altitude: {data['altitude_m']} m"
            )
            time.sleep(5)
    except KeyboardInterrupt:
        print("Environment sensor stopped.")

if __name__ == "__main__":
    main()