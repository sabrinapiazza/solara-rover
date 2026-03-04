import board
import adafruit_bno055
import busio

print(dir(board))  #output should show SCL and SDA

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

print([a for a in dir(sensor) if not a.startswith('_')])

# print("Quaternion:", sensor.quaternion)
# print("Gyro:", sensor.gyroscope)
# print("Accel:", sensor.acceleration)