import bme280
from machine import SoftI2C, I2C, Pin

# i2c = I2C(0, freq=400000, scl=Pin(1), sda=Pin(0))
i2c = SoftI2C(sda=Pin(0), scl=Pin(1), freq=400000) 
print(i2c.readfrom_mem(0x76, 0xd0, 1)[0])

# devices = i2c.scan()
# for device in devices: print(device)
# pth = bme280.BME280(i2c=i2c)
