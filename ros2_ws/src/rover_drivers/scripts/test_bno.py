import board
import adafruit_bno055

print(dir(board))  #output should show SCL and SDA

# i2c = board.I2C()
# sensor = adafruit_bno055.BNO055_I2C(i2c)

# print("Quaternion:", sensor.quaternion)
# print("Gyro:", sensor.gyroscope)
# print("Accel:", sensor.acceleration)