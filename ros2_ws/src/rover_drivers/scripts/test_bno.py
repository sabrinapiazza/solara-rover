import board
import adafruit_bno055
import busio

print(dir(board))  #output should show SCL and SDA

i2c = busio.I2C(board.SDA, board.SCL)
sensor = adafruit_bno055.BNO055(i2c)

print("Quaternion:", sensor.quaternion)
print("Gyro:", sensor.gyroscope)
print("Accel:", sensor.acceleration)