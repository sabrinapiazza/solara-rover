# BME280 sensor driver
# Reads temperature, humidity, pressure, air quality
# Returns environmental measurements
# https://www.rp2040learning.com/code/circuitpython/raspberry-pi-pico-and-bme680-gas-sensor-circuitpython-example.php
# https://docs.circuitpython.org/projects/bme680/en/latest/api.html#implementation-notes
# https://docs.circuitpython.org/projects/bme680/en/latest/

# import adafruit_bme680
import adafruit_bme280.basic as adafruit_bme280
import time
import board    #breakout-specific pin identities
import busio

SEA_LEVEL_PRESSURE_HPA = 1020.7  # hPa - adjust to local sea-level pressure
                                  # check for riverside since this r'garden

# You will usually have to add an offset to account for the temperature of
# the sensor. This is usually around 5 degrees but varies by use. Use a
# separate temperature sensor to calibrate this one.
TEMPERATURE_OFFSET = -5


def get_data():
    # Single read of all BME280 data. Returns a dict for collector.py.

    # sudo raspi-config
    # sudo apt install i2c-tools
    # reboot after enabling I2C
    # pip3 install -r requirements.txt
    # pip3 install --upgrade adafruit_blinka
    # install with pip3 but run with python rather than python3

    # python3 -c "import board"
    # ls /dev/i2c*      (should see /dev/i2c-1)
    # i2cdetect -y 1

    # i2c = board.I2C()
    i2c = busio.I2C(board.SCL, board.SDA)

    # To initialise using the default address:
    # bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)

    # change this to match the location's pressure (hPa) at sea level
    # This standard pressure value lets the sensor estimate altitude.
    # If you know your local sea-level pressure (from weather reports), you can adjust this for more accurate altitude readings.
    # bme680.sea_level_pressure = 1020.7   #check for riverside since this r'garden
    bme280.sea_level_pressure = SEA_LEVEL_PRESSURE_HPA

    return {
        "temperature_c": round(bme280.temperature + TEMPERATURE_OFFSET, 2),  # degrees Celsius
        # "gas_ohms":    bme680.gas,  # gas resistance in ohms
        "humidity_rh":   round(bme280.relative_humidity, 2),                  # RH %
        "pressure_hpa":  round(bme280.pressure, 2),                           # hectoPascals
        "altitude_m":    round(bme280.altitude, 2),                           # meters
    }


def main():
    # Create sensor object, communicating over the board's default I2C bus
    print(board.__file__)
    print(dir(board))

    # Every 2 seconds, it reads all five values from the sensor
    # and prints them formatted to specific decimal places (like %0.1f for one decimal place).
    # The sensor object handles all the low-level I2C communication—you just access properties like bme680.temperature and it fetches the data from the chip for you.
    while True:
        data = get_data()
        print(data)
        time.sleep(5)


if __name__ == "__main__":
    main()