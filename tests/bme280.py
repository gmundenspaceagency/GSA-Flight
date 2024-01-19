from smbus2 import SMBus
import bme280
    
class BME:
    def __init__(self, bus_number:int=1, address=0x76)->None:
        self.bus = SMBus(bus_number)
        self.address = address
        self.calibration_params = bme280.load_calibration_params(self.bus, self.address)
    
    def read_data(self)->bme280.compensated_readings:
        data = bme280.sample(self.bus, self.address, self.calibration_params)
        return data