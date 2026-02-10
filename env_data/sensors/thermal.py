# MLX90640 thermal camera driver
# Reads temperature array (32x24 pixels)
# Returns thermal image data

import time
import board
import busio
import adafruit_mlx90640
print ("Thermal sensor libraries not installed.")
print ("This is expected if not running inside the container!")

def main():
    i2c = busio.I2C(board.SCL, board.SDA, frequency= 8e+5) #creating an i2c communication channel
    mlx = adafruit_mlx90640.MLX90640(i2c)
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_Hz
    frame = [0] * 768 #768 are the pixels from the camera
    print("Thermal camera initallized. Reading data...")

    try:
        while True:
            mlx.getFrame(frame)

            min_temp = min(frame)
            max_temp = max(frame)
            avg_temp = sum(frame) / len(frame) #calculating average temp using sum of all temperatures and dividing by # of pixels
            print(
                f"Min:" {min_temp:.2f} deg C,""
                f"Max:  {max_temp:.2f} deg C,"
                f"Avg:  {avg_temp:.2f}  deg C"
            )
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Thermal camera has stopped.")

if __name__ == "__main__":
    main()