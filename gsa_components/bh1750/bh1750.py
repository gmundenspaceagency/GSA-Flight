import time
from smbus2 import SMBus
from time import sleep

class Bh1750:
    """Standard Python BH1750 ambient light sensor driver."""

    PWR_OFF = 0x00
    PWR_ON = 0x01
    RESET = 0x07

    # modes
    CONT_LOWRES = 0x13
    CONT_HIRES_1 = 0x10
    CONT_HIRES_2 = 0x11
    ONCE_HIRES_1 = 0x20
    ONCE_HIRES_2 = 0x21
    ONCE_LOWRES = 0x23

    def __init__(self, bus_number=1, addr=0x23):
        self.bus = SMBus(bus_number)
        self.addr = addr
        self.off()
        self.reset()
    
    def off(self):
        """Turn sensor off."""
        self.set_mode(self.PWR_OFF)

    def on(self):
        """Turn sensor on."""
        self.set_mode(self.PWR_ON)

    def reset(self):
        """Reset sensor, turn on first if required."""
        self.on()
        self.set_mode(self.RESET)

    def set_mode(self, mode):
        """Set sensor mode."""
        self.mode = mode
        self.bus.write_byte(self.addr, self.mode)

    def luminance(self, mode):
        """Sample luminance (in lux), using specified sensor mode."""
        # continuous modes
        if mode & 0x10 and mode != self.mode:
            self.set_mode(mode)
        # one-shot modes
        if mode & 0x20:
            self.set_mode(mode)
        # earlier measurements return the previous reading
        sleep(0.024 if mode in (0x13, 0x23) else 0.180)
        data = self.bus.read_i2c_block_data(self.addr, 0x20, 2)
        factor = 2.0 if mode in (0x11, 0x21) else 1.0
        return (data[0] << 8 | data[1]) / (1.2 * factor)

if __name__ == '__main__':
    bh1750 = Bh1750()

    while True:
        luminance = bh1750.luminance(Bh1750.CONT_LOWRES)
        print('luminance: %f lux' % luminance)
        time.sleep(0.1)
