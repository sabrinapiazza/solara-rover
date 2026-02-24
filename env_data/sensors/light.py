# TSL2591 light sensor driver
# Reads lux (light intensity)
# Returns brightness level

import time
import board
import adafruit_tsl2591
import busio

def get_data():
    # Single read of all TSL2591 sensors. Returns a dict for collector.py.
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_tsl2591.TSL2591(i2c)

    sensor.gain = adafruit_tsl2591.GAIN_MED          # 25x gain (default)
    sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS

    return {
        "lux":           round(sensor.lux, 4),   # total light in lux
        "infrared":      sensor.infrared,         # IR only,  0-65535 (16-bit)
        "visible":       sensor.visible,          # visible only, 0-2147483647 (32-bit)
        "full_spectrum": sensor.full_spectrum,    # visible + IR, 0-2147483647 (32-bit)
    }

def main():
    # Standalone loop for testing light.py directly.
    print("TSL2591 initialized. Reading data...")
    try:
        while True:
            data = get_data()
            print(
                f"Lux: {data['lux']}, "
                f"Infrared: {data['infrared']}, "
                f"Visible: {data['visible']}, "
                f"Full Spectrum: {data['full_spectrum']}"
            )
            time.sleep(5)
    except KeyboardInterrupt:
        print("Light sensor stopped.")

if __name__ == "__main__":
    main()