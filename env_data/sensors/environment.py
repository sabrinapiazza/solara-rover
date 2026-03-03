import time
import board
import adafruit_bme680

def main():
    # On Raspberry Pi, board.I2C() automatically uses /dev/i2c-1
    i2c = board.I2C()  

    # Initialize BME680 using I2C
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)

    # Set your local sea-level pressure for accurate altitude
    bme680.sea_level_pressure = 1020.7

    while True:
        print("Temperature:", bme680.temperature)
        print("Humidity:", bme680.relative_humidity)
        print("Pressure:", bme680.pressure)
        print("Gas:", bme680.gas)
        print("Altitude:", bme680.altitude)
        print("--------------------")
        time.sleep(5)

if __name__ == "__main__":
    main()