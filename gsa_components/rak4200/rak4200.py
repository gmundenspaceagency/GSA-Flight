import os
import time
import serial
from datetime import datetime
from typing import Optional

ERROR_CODES = {
    1: 'The last command received is an unsupported AT command.',
    2: 'Invalid parameter in the AT command.',
    3: 'There is an error when reading or writing the flash memory.',
    5: 'There is an error when sending data through the UART port. Check if you exceed 256 bytes UART buffer.',
    80: 'The LoRa transceiver is busy, could not process a new command.',
    81: 'LoRa service is unknown. Unknown MAC command received by the node. Execute commands that are not supported in the current state, such as sending the at+join command in P2P mode.',
    82: 'The LoRa parameters are invalid.',
    83: 'The LoRa frequency is invalid.',
    84: 'The LoRa data rate (DR) is invalid.',
    85: 'The LoRa frequency and data rate are invalid.',
    86: 'The device hasn’t joined into a LoRa network.',
    87: 'The length of the packet exceeded the maximum allowed by the LoRa protocol.',
    88: 'Service is closed by the server. Due to the limitation of the duty cycle, the server will send the "SRV_MAC_DUTY_CYCLE_REQ" MAC command to close the service.',
    89: 'This is an unsupported region code.',
    90: 'Duty cycle is restricted. Due to the duty cycle, data cannot be sent at this time until the time limit is removed.',
    91: 'No valid LoRa channel could be found.',
    92: 'No available LoRa channel could be found.',
    93: 'Status is error. Generally, the internal state of the protocol stack is wrong.',
    94: 'Time out reached while sending the packet through the LoRa transceiver.',
    95: 'Time out reached while waiting for a packet in the LoRa RX1 window.',
    96: 'Time out reached while waiting for a packet in the LoRa RX2 window.',
    97: 'There is an error while receiving a packet during the LoRa RX1 window.',
    98: 'There is an error while receiving a packet during the LoRa RX2 window.',
    99: 'Failed to join into a LoRa network.',
    100: 'Duplicate downlink message is detected. A message with an invalid downlink count is received.',
    101: 'Payload size is not valid for the current data rate (DR).',
    102: 'Many downlink packets are lost.',
    103: 'Address fail. The address of the received packet does not match the address of the current node.',
    104: 'Invalid MIC is detected in the LoRa message.'
}

class Rak4200:
    def __init__(self:any, baudrate=115200, serial_port='/dev/ttyS0')->None:
        if not os.path.exists(serial_port):
            raise RuntimeError(f'Device not connected at {serial_port}')

        self.baudrate = baudrate
        self.serial_port = serial_port
        self.uart0 = serial.Serial(serial_port, baudrate=baudrate, timeout=1)
        # wait for serial port connection to be established
        time.sleep(1)

    def sendAtCMD(self:any, at_cmd:str)->Optional[str]:
        self.uart0.write((at_cmd + '\r\n').encode())
        dataString = self.uart0.readline().decode('utf-8', errors='replace')
        return dataString if dataString != '' else None
    
    def parse_response(self:any, response:str, allow_empty:bool=False)->str:
        if response is None:
            if allow_empty:
                return ''
            else:
                raise RuntimeError(f'Could not establish connection to RAK4200 over serial port {self.serial_port} with baudrate {self.baudrate}')

        if 'OK' in response:
            return response.replace('OK ', '')
        else:
            if 'ERROR: ' in response:
                error_code = int(response.replace('ERROR: ', ''))
                error = ERROR_CODES[error_code]
                raise RuntimeError(f'RAK4200-Error {error_code}: {error}')

            raise RuntimeError(f'Unknown response from RAK4200: {response}')

    def set_mode(self:any, mode:str)->None:
        if mode not in ['send', 'receive']:
            raise ValueError('Mode must be "send" or "receive"')
        
        self.mode = mode
        mode_number = '1' if mode == 'receive' else '2'
        response = self.sendAtCMD(f'at+set_config=lorap2p:transfer_mode:{mode_number}')
        self.parse_response(response)

    def set_workmode_p2p(self:any)->None:
        response = self.sendAtCMD('at+set_config=lora:work_mode:1')
        self.parse_response(response, allow_empty=True)

        # wait for module restart
        time.sleep(1)

    def set_p2p_config(self:any, frequency:int=869.525, spreadfact:int=12, bandwidth:int=0, codingrate:int=1, preamble_length:int=8, tx_power:int=20):
        """
        Configures parameters for point-to-point communication.

        Parameters:
        - frequency (int): Frequency in Mega-Hertz (MHz). Default is 869.525 MHz.
        - spreadfact (int): Spreading factor. Default is 12.
        - bandwidth (int): Bandwidth in kHz. Options: 0 (125 kHz), 1 (250 kHz), 2 (500 kHz). Default is 0.
        - codingrate (int): Coding rate options: 1 (4/5), 2 (4/6), 3 (4/7), 4 (4/8). Default is 1.
        - preamble_length (int): Preamble length. Range: 5 to 65535. Default is 8.
        - tx_power (int): Transmit power in dBm. Range: 5 to 20. Default is 20.

        Returns:
        None
        """ 

        response = self.sendAtCMD(f'at+set_config=lorap2p:{int(frequency * 1_000_000)}:{spreadfact}:{bandwidth}:{codingrate}:{preamble_length}:{tx_power}')
        self.parse_response(response)

    def get_version(self:any)->str:
        response = self.sendAtCMD(f'at+version')
        return self.parse_response(response)

    def start(self:any, mode:str)->None:
        self.set_mode(mode)
        # Setting bandwidth to 1 (250kHz) for faster data transmission and spread factor to 7 (obligatory in Europe when using 250kHz)
        # self.set_p2p_config(bandwidth=1, spreadfact=7)
        currentTime = datetime.now()
        currentTimeStr = currentTime.strftime('%H:%M:%S')

        if self.mode == 'send':
            self.send('(Info) RAK4200 is ready at: ' + currentTimeStr)
        
        # wait for module to process information
        time.sleep(1)
    
    def send(self:any, data:str)->None:
        if self.mode != 'send':
            self.set_mode('send')

        # self.uart0.write(f'at+send=lorap2p:{data.encode('utf-8').hex()}\r\n') ??
        self.uart0.write(('at+send=lorap2p:' + data.encode('utf-8').hex() + '\r\n').encode())

    def receive(self:any)->Optional[dict]:
        if self.mode != 'receive':
            self.set_mode('receive')
        
        received_data = self.uart0.readline()

        if received_data != b'':
            try:
                decoded_data = received_data.decode('utf-8')

                if 'ERROR: ' in decoded_data:
                    error_code = int(decoded_data.replace('ERROR: ', ''))
                    error = ERROR_CODES[error_code]
                    print(f'RAK4200-Error {error_code}: {error}')
            
                data_str = decoded_data.split('=')[1]
                data_vals = data_str.split(',')
                signal_strength = int(data_vals[0])
                noise = int(data_vals[1])
                message_info = data_vals[2].split(':')
                message_length = int(message_info[0])
                message_hex = message_info[1]
                message = bytes.fromhex(message_hex).decode('utf-8')
                
                return {
                    'signal_strength': signal_strength,
                    'noise': noise,
                    'message_length': message_length,
                    'message': message,
                }
            except Exception as error:
                print(f'Error while processing message "{received_data}": {str(error)}')         


if __name__ == '__main__':
    test_mode = 'send'

    if test_mode == 'receive':
        test = Rak4200(serial_port='/dev/ttyUSB0')
        test.start('receive')
        print('RAK4200 connected, receiving messages...')

        while True:
            data = test.receive()

            if data is not None:
                print(data)

    if test_mode == 'send':
        test = Rak4200(serial_port='/dev/ttyS0')
        test.set_mode('send')
        print('RAK4200 connected, sending test data...')
    
        while True:
            currentTime = datetime.now()
            currentTimeStr = currentTime.strftime('%H:%M:%S')
            message = 'RAK4200 test at: ' + currentTimeStr
            test.send(message)
            print(f'Sent: {message}')
            time.sleep(1)