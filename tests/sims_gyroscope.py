from mpu6050 import mpu6050
import time

# Replace '0x68' with the correct I2C address if needed
sensor = mpu6050(0x68)

try:
    rotation = 0
    while True:
        fixed_average_sensor_drift = 0.86
        rotation += (sensor.get_gyro_data()['z'] - fixed_average_sensor_drift) / 10
        print(round(rotation))
        time.sleep(0.1)
        # s = 0
        # for i in range(0, 10000):
            # s += sensor.get_gyro_data()['z']
        # print(s / 1000)
        #print(rotation)
except KeyboardInterrupt:
    print("Program terminated by user.")
