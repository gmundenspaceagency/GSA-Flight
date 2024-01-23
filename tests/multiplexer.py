import smbus
from time import sleep
import sys
sys.path.append('..')
from gsa_components.bh1750 import Bh1750
light_sensor = Bh1750(addr=0x5c)

channel_array=[0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000,0b10000000]

def I2C_setup(multiplexer, i2c_channel_setup):
    bus = smbus.SMBus(1)
    bus.write_byte(multiplexer,channel_array[i2c_channel_setup])
    sleep(0.1)
    print("TCA9548A I2C channel status:", bin(bus.read_byte(multiplexer)))

I2C_setup(0x70, 7)
print(light_sensor.luminance(Bh1750.ONCE_HIRES_1))
# Read data from the BH1750 sensor


