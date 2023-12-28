from machine import I2C, Pin
from bh1750 import BH1750
import time

# Initialize I2C-Pins
i2c_sda = Pin(20)
i2c_scl = Pin(21)

bh1750 = BH1750(I2C(0, sda=i2c_sda,scl=i2c_scl))

while True:
    luminance = bh1750.luminance(BH1750.ONCE_HIRES_1)
    print("luminance: %f lux" % luminance)
    time.sleep(0.5)

main()
