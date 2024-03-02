import smbus
from time import sleep
import os
import subprocess
import re

class Multiplexer:
    MAX_CHANNEL = 7
    CHANNEL_ARRAY=[0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000,0b10000000]

    def __init__(self:any, address=0x70, busnr:int=1)->None:
        self.address = address
        self.busnr = busnr
        self.bus = smbus.SMBus(busnr)

    def switch_to(self:any, channel:int)->None:
        if channel < 0 or channel > self.MAX_CHANNEL:
            raise ValueError(f"Channel must be between 0 and {self.MAX_CHANNEL}")
    
        self.bus.write_byte(self.address, self.CHANNEL_ARRAY[channel])

        # wait for byte to be written (using oddly specific time to seem scientific)
        sleep(0.00002311)
    
    def scan_addresses(self:any)->None:
        for channel in range(self.MAX_CHANNEL):
            self.switch_to(channel)
            p = subprocess.Popen(['i2cdetect', '-y',str(self.busnr)],stdout=subprocess.PIPE,)
            line = str(p.stdout.readline())
            addrs = []

            for match in re.finditer("[0-9][0-9]:.*[0-9][0-9]", line):
                addrs.append(match.group())
            
            print(f"Found on channel f{channel}: {', '.join(addrs)}")

if __name__ == '__main__':
    test = Multiplexer(bus=smbus.SMBus(1))
    test.scan_addresses()
