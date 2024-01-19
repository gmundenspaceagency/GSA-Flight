import serial
import time
from datetime import datetime
from typing import Optional

class Rak4200:
    def __init__(self:any, baudrate=115200, serial_port='/dev/ttyS0')->None:
        # Konfiguration des seriellen Ports
        self.baudrate = baudrate
        self.uart0 = serial.Serial(serial_port, baudrate=baudrate, timeout=0)

    # Funktion zum Senden von AT-Kommandos
    def sendCmdAT(self:any, uart:serial.Serial, at_cmd:str, wait:int=1)->Optional[str]:
        uart.write((at_cmd + '\r\n').encode())
        time.sleep(1)
        dataString = uart.readline().decode('utf-8', errors='replace')

        return dataString if dataString != '' else None
    
    def start(self:any)->None:
        if self.sendCmdAT(self.uart0, 'at+set_config=lorap2p:transfer_mode:2') is None:
            raise SystemError('Could not establish connection to RAK4200')
        
        currentTime = datetime.now()
        currentTimeStr = currentTime.strftime("%H:%M:%S")
        
        self.send('(Info) RAK4200 is ready at: ' + currentTimeStr)
    
    def send(self:any, data:str)->None:
        self.uart0.write(("at+send=lorap2p:" + data.encode('utf-8').hex() + "\r\n").encode())

if __name__ == '__main__':
    test = Rak4200()
    test.start()
    print('RAK4200 connected!')
