# BME680 sensor driver
# Reads temperature, humidity, pressure, air quality
# Returns environmental measurements
# https://www.rp2040learning.com/code/circuitpython/raspberry-pi-pico-and-bme680-gas-sensor-circuitpython-example.php
# https://docs.circuitpython.org/projects/bme680/en/latest/api.html#implementation-notes
# https://docs.circuitpython.org/projects/bme680/en/latest/

import adafruit_bme680
import time
import board    #breakout-specific pin identities

# sudo raspi-config
# sudo apt install i2c-tools
# reboot after enabling I2C
# pip3 install -r requirements.txt
# pip3 install --upgrade adafruit_blinka
# install with pip3 but run with python rather than python3

# python3 -c "import board"
# ls /dev/i2c*      (should see /dev/i2c-1)
# i2cdetect -y 1

def main():
    # Create sensor object, communicating over the board's default I2C bus
    # This connects the BME680 to your microcontroller using I2C (Inter-Integrated Circuit), a common 2-wire communication protocol. 
    # The board.I2C() automatically finds the correct pins for your specific board (like a Raspberry Pi Pico, ESP32, or Arduino).
    # Use the raw pin IDs for a Raspberry Pi
    # GPIO 2 is SDA, GPIO 3 is SCL
    # try:
    #     i2c = busio.I2C(microcontroller.pin.GPIO3, microcontroller.pin.GPIO2)
    #     print("Manual I2C initialization successful!")
    # except Exception as e:
    #     print(f"Hardware Access Error: {e}")
    #     return

    # # Now continue with your BME680 setup
    # try:
    #     bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
    # except Exception as e:
    #     print(f"Could not find BME680: {e}")
    i2c = busio.I2C(board.SCL, board.SDA)

    # To initialise using the default address:
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, 0x76, True)

    # change this to match the location's pressure (hPa) at sea level
    # This standard pressure value lets the sensor estimate altitude. 
    # If you know your local sea-level pressure (from weather reports), you can adjust this for more accurate altitude readings.
    bme680.sea_level_pressure = 1020.7   #check for riverside since this r'garden

    # Every 2 seconds, it reads all five values from the sensor 
    # and prints them formatted to specific decimal places (like %0.1f for one decimal place).
    # The sensor object handles all the low-level I2C communication—you just access properties like bme680.temperature and it fetches the data from the chip for you.
    bme_input = {}

    while True:
        bme_input["Temperature"] = bme680.temperature  #The compensated temperature in degrees Celsius
        bme_input["Gas"] = bme680.gas  #The gas resistance in ohms
        bme_input["Humidity"] = bme680.relative_humidity  #The relative humidity in RH %
        bme_input["Pressure"] = bme680.pressure  #The barometric pressure in hectoPascals
        bme_input["Altitude"] = bme680.altitude  #The altitude based on current pressure vs the sea level pressure (sea_level_pressure) - which you must enter ahead of time)
        
        time.sleep(5)
        
if __name__ == "__main__":
    main()