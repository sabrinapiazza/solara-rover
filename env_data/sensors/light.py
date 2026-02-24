# TSL2591 light sensor driver
# Reads lux (light intensity)
# Returns brightness level
import time
import board
import adafruit_tsl2591
import busio

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_tsl2591.TSL2591(i2c)

    #gain value (sensitivity)
    sensor.gain = adafruit_tsl2591.GAIN_MED
    #GAIN_LOW (1x gain)
    #GAIN_MED (25x gain, default)
    #GAIN_HIGH (428x gain)
    #GAIN_MAX (9876x gain)

    #integration time
    sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS
    #100MS-600MS

    # Read the total lux, IR, and visible light levels and print it every second.
    while True:
        # Read and calculate the light level in lux.
        lux = sensor.lux
        print(f"Total light: {lux}lux")
        # You can also read the raw infrared and visible light levels.
        # These are unsigned, the higher the number the more light of that type.
        # There are no units like lux.
        # Infrared levels range from 0-65535 (16-bit)
        infrared = sensor.infrared
        print(f"Infrared light: {infrared}")
        # Visible-only levels range from 0-2147483647 (32-bit)
        visible = sensor.visible
        print(f"Visible light: {visible}")
        # Full spectrum (visible + IR) also range from 0-2147483647 (32-bit)
        full_spectrum = sensor.full_spectrum
        print(f"Full spectrum (IR + visible) light: {full_spectrum}")
        #1 second delay
        time.sleep(5)
    
    
if __name__ == "__main__": #running when file is executed *directly
    main()