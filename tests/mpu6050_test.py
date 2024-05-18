from mpu6050 import mpu6050
from time import sleep

sensor = mpu6050(0x68)
sensor.set_accel_range(sensor.ACCEL_RANGE_16G)
i = 0
ax = 0
ay = 0
az = 0

while True:
    i += 1
    data = sensor.get_accel_data()
    ax = (ax * (i - 1) + data['x']) / i
    ay = (ay * (i - 1) + data['y']) / i
    az = (az * (i - 1) + data['z']) / i
    print(ax, ay, az)
    sleep(0.01)
