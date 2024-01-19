from machine import Pin, I2C
import time

# MPU6050 I2C address
MPU6050_I2C_ADDR = 0x68

# MPU6050 register addresses
MPU6050_REG_PWR_MGMT_1 = 0x6B
MPU6050_REG_ACCEL_XOUT_H = 0x3B
MPU6050_REG_ACCEL_XOUT_L = 0x3C
MPU6050_REG_ACCEL_YOUT_H = 0x3D
MPU6050_REG_ACCEL_YOUT_L = 0x3E
MPU6050_REG_ACCEL_ZOUT_H = 0x3F
MPU6050_REG_ACCEL_ZOUT_L = 0x40

# Initialize I2C
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)

# MPU6050 initialization
i2c.writeto_mem(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, bytes([0]))

while True:
    # Read Z-axis acceleration data
    z_high = i2c.readfrom_mem(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_ZOUT_H, 1)[0]
    z_low = i2c.readfrom_mem(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_ZOUT_L, 1)[0]

    # Combine high and low bytes
    z_value = (z_high << 8) | z_low

    # Convert to signed 16-bit value (since MicroPython doesn't support native int16)
    if z_value & 0x8000:
        z_value -= 0x10000

    print("Z-axis acceleration:", z_value)

    # Delay for a short interval (adjust as needed)
    time.sleep(1)