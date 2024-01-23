import serial
import time
import os
from typing import Optional

# TODO: use rak4200 class to do this
# Don't sleep between listen but find out when a message starts and when it ends
# Extend the rak4200 class to handle settings of frequency and co, also better communication with sensor like wait for ready and restart (read documentation)
# Add Error handling to transmitting and receiving
# Do cool stuff with the received data (and make sure the program can't break from receiving defect transmissions)
# We need a .gitignore for _pycache_ and co
# Access my VM over remote desktop, vpn?

cansat_id = '69xd'

def is_device_connected(device_path):
    return os.path.exists(device_path)

def send_command(serial_port, command):
    serial_port.write(command.encode('utf-8'))
    time.sleep(1)  # Wait for the command to be processed

def sendCmdAT(uart:serial.Serial, at_cmd:str, wait:int=1)->Optional[str]:
    uart.write((at_cmd + '\r\n').encode())
    time.sleep(1)
    dataString = uart.readline().decode('utf-8', errors='replace')

    return dataString if dataString != '' else None

device_path = '/dev/ttyUSB0'

if is_device_connected(device_path):
    try:
        with serial.Serial(device_path, 115200, timeout=None) as ser:
            print(f"Connected to {device_path}")

            command_response = sendCmdAT(ser, 'at+set_config=lorap2p:transfer_mode:1')
            if command_response is None:
                raise SystemError('Could not establish connection to RAK4200')


            while True:
                received_data = ser.read_until(b'\n')

                try:
                    if received_data != b'':
                        decoded_data = received_data.decode('utf-8')
                        data_str = decoded_data.split('=')[1]
                        data_vals = data_str.split(',')
                        signal_strength = data_vals[0]
                        noise = data_vals[1]
                        message_info = data_vals[2].split(':')
                        message_length = message_info[0]
                        message_hex = message_info[1]
                        message = bytes.fromhex(message_hex).decode('utf-8')

                        if not '(Info)' in message:
                            message_vals = message.split(';')
                            [cansat_id, timestamp, pressure, temperature] = message_vals
                            print(f'CanSat-ID: {cansat_id}, Timestamp: {timestamp}, Pressure: {pressure}hPa, Temperature: {temperature}°C')
                        else:
                            print(message)
                except Exception as error:
                    print(f'Error while processing message "{received_data}": {str(error)}')           

    except serial.SerialException as e:
        print(f"Error: {e}")
else:
    print(f"Device not connected at {device_path}")