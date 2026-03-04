import board
import adafruit_bno055
import busio

print(dir(board))  #output should show SCL and SDA

i2c = board.I2C()
sensor = adafruit_bno055.BNO055(i2c)

print("Quaternion:", sensor.quaternion)
print("Gyro:", sensor.gyroscope)
print("Accel:", sensor.acceleration)