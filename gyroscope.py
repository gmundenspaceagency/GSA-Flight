import smbus
import math
from time import sleep

class GYRO:
    def __init__(self, bus_number: int = 1, address=0x68):
        # Register
        self.power_mgmt_1 = 0x6b
        self.power_mgmt_2 = 0x6c
        
        self.bus = smbus.SMBus(1)
        self.address = 0x68
        
        # Aktivieren, um das Modul ansprechen zu koennen
        self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)
     
    def read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)
     
    def read_word(self, reg):
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg+1)
        value = (h << 8) + l
        return value
     
    def read_word_2c(self, reg):
        val = self.read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
     
    def dist(self, a, b):
        return math.sqrt((a*a)+(b*b))
     
    def get_y_rotation(self, x, y, z):
        radians = math.atan2(x, self.dist(y, z))
        return round(-math.degrees(radians), 2)
     
    def get_x_rotation(self, x, y, z):
        radians = math.atan2(y, self.dist(x, z))
        return round(math.degrees(radians), 2)
    
    def get_rotation(self, x, y, z):
        xradians = math.atan2(y, self.dist(x, z))
        xrotation = round(math.degrees(xradians), 2)
        yradians = math.atan2(x, self.dist(y, z))
        yrotation = round(-math.degrees(yradians), 2)
        return xrotation, yrotation
    
    def get_scaled_gyroscope(self):
        gyroskop_xout = self.read_word_2c(0x43)
        gyroskop_yout = self.read_word_2c(0x45)
        gyroskop_zout = self.read_word_2c(0x47)
        
        # random subtraction to correct sensor specific drift
        gyroskop_xout_scaled = round(gyroskop_xout / 131.0 - 20, 2)
        gyroskop_yout_scaled = round(gyroskop_yout / 131.0 - 1, 2)
        gyroskop_zout_scaled = round(gyroskop_zout / 131.0 - 1, 2)
        
        return gyroskop_xout_scaled, gyroskop_yout_scaled, gyroskop_zout_scaled
    
    def get_scaled_acceleration(self):
        acceleration_xout = self.read_word_2c(0x3b)
        acceleration_yout = self.read_word_2c(0x3d)
        acceleration_zout = self.read_word_2c(0x3f)
        
        acceleration_xout_scaled = round(acceleration_xout / 16384.0, 2)
        acceleration_yout_scaled = round(acceleration_yout / 16384.0, 2)
        acceleration_zout_scaled = round(acceleration_zout / 16384.0, 2)
        
        return acceleration_xout_scaled, acceleration_yout_scaled, acceleration_zout_scaled
