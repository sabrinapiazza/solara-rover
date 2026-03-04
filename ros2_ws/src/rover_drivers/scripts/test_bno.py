import board
import adafruit_bno055

i2c = board.I2C()
sensor = adafruit_bno055.Adafruit_BNO055_I2C(i2c)

print("Quaternion:", sensor.quaternion)
print("Gyro:", sensor.gyroscope)
print("Accel:", sensor.acceleration)