import board
import adafruit_bno055
import busio
import time

print(dir(board))  #output should show SCL and SDA

i2c = busio.I2C(board.SCL, board.SDA)
time.sleep(0.5)  # give sensor time to settle

sensor = adafruit_bno055.BNO055_I2C(i2c)
time.sleep(0.5)  # give sensor time to settle

# print([a for a in dir(sensor) if not a.startswith('_')])

print("Quaternion:", sensor.quaternion)
print("Gyro:", sensor.gyro)
print("Accel:", sensor.acceleration)