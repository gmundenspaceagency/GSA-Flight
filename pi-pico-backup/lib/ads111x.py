from machine import I2C, Pin

class ADS111X:
    def __init__(self, scl_pin:int, sda_pin:int)->None:
        self.dev = I2C(1, freq=400000, scl=Pin(scl_pin), sda=Pin(sda_pin))
        self.address = 72
 
    def read_config(self):
        self.dev.writeto(self.address, bytearray([1]))
        result = self.dev.readfrom(self.address, 2)
        return result[0] << 8 | result[1]
 
    def read_value(self):
        self.dev.writeto(self.address, bytearray([0]))
        result = self.dev.readfrom(self.address, 2)
        config = self.read_config()
        config &= ~(7 << 12) & ~(7 << 9)
        config |= (4 << 12) | (1 << 9) | (1 << 15)
        config = [int(config >> i & 0xff) for i in (8, 0)]
        self.dev.writeto(self.address, bytearray([1] + config))
        return result[0] << 8 | result[1]
 
    def val_to_voltage(self, val, max_val=26100, voltage_ref=3.3):
        return val / max_val * voltage_ref
    
    def info(self):
        print(bin(self.read_config()))
