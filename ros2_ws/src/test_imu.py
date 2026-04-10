import time, smbus2
from adafruit_bno055 import BNO055_I2C

class I2CBus:
    def __init__(self, bus_num):
        self._bus = smbus2.SMBus(bus_num)
    def try_lock(self): return True
    def unlock(self): pass
    def scan(self): return []
    def writeto(self, addr, buf, **kwargs):
        if len(buf) == 0:
            try:
                self._bus.read_byte(addr)
            except:
                pass
        else:
            self._bus.write_i2c_block_data(addr, buf[0], list(buf[1:]))
    def readfrom_into(self, addr, buf, **kwargs):
        for i in range(len(buf)):
            buf[i] = self._bus.read_byte(addr)
    def writeto_then_readfrom(self, addr, out, buf,
                           out_start=0, out_end=None,
                           in_start=0, in_end=None, **kwargs):
        out_end  = out_end or len(out)
        in_end   = in_end  or len(buf)
        read_len = in_end - in_start
        reg = out[out_start]
        for i in range(read_len):
            for attempt in range(5):  # retry up to 5 times
                try:
                    buf[in_start + i] = self._bus.read_byte_data(addr, reg + i)
                    break
                except OSError:
                    time.sleep(0.01)  # wait 10ms and retry
                    
                    
# IMU LOGIC NOW MOVED INSIDE RUN_TEST_LOOP()
i2c = I2CBus(4)  # BNO055 on i2c-4
sensor = BNO055_I2C(i2c, address=0x28)

while True:
    imu_data = {
        "orientation":         {"x": sensor.quaternion[1],   "y": sensor.quaternion[2],   "z": sensor.quaternion[3],   "w": sensor.quaternion[0]},
        "angular_velocity":    {"x": sensor.gyro[0],         "y": sensor.gyro[1],         "z": sensor.gyro[2]},
        "linear_acceleration": {"x": sensor.acceleration[0], "y": sensor.acceleration[1], "z": sensor.acceleration[2]},
    }
    print(f"  imu:       {imu_data}")
    time.sleep(5)
    
