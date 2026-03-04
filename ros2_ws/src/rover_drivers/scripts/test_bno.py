import board
import adafruit_bno055
import busio
from adafruit_extended_bus import ExtendedI2C as I2C

print(dir(board))  #output should show SCL and SDA

# i2c = board.I2C()
i2c = I2C(1)
sensor = adafruit_bno055.BNO055(i2c)

print("Quaternion:", sensor.quaternion)
print("Gyro:", sensor.gyroscope)
print("Accel:", sensor.acceleration)