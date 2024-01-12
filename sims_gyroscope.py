from mpu6050 import mpu6050
import time

# Replace '0x68' with the correct I2C address if needed
sensor = mpu6050(0x68)

def get_z_rotation():
    gyro_data = sensor.get_gyro_data()
    z_rotation = gyro_data['z']
    return z_rotation

try:
    while True:
        z_rotation = get_z_rotation()
        print(f"Z Rotation: {z_rotation} degrees")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Program terminated by user.")
