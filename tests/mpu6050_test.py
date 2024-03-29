from mpu6050 import mpu6050
from time import sleep

sensor = mpu6050(0x68)
sensor.set_accel_range(sensor.ACCEL_RANGE_16G)
max_accel = 0

while True:
    data = sensor.get_accel_data()
    max_accel = max(max_accel, round(data['z']/10, 2))
    print(max_accel, *[round(val/10, 2) for val in [data['x'], data['y'], data['z']]])
    sleep(0.01)
