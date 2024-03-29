from time import sleep, perf_counter
import sys
sys.path.append('..')
sys.path.append('.')
from gsa_components.rak4200 import Rak4200
import platform

mode = 'send' if platform.machine() == 'armv6l' else 'receive'

"""
Parameters:
    - frequency (int): Frequency in Mega-Hertz (MHz). Default is 869.525 MHz.
    - spreadfact (int): Spreading factor. Default is 12.
    - bandwidth (int): Bandwidth in kHz. Options: 0 (125 kHz), 1 (250 kHz), 2 (500 kHz). Default is 0.
    - codingrate (int): Coding rate options: 1 (4/5), 2 (4/6), 3 (4/7), 4 (4/8). Default is 1.
    - preamble_length (int): Preamble length. Range: 5 to 65535. Default is 8.
    - tx_power (int): Transmit power in dBm. Range: 5 to 20. Default is 20.
"""
parameters = (860.525, 7, 0, 4, 64, 20)

send_timeout = 0.5 # only for sending

if mode == 'receive':
    rak = Rak4200(serial_port='/dev/ttyUSB0')
    rak.set_p2p_config(*parameters)
    rak.start('receive')
    print('RAK4200 connected, receiving messages...')
    received = last_index = start_index = 0
    strenghts = []
    noises = []
    start = perf_counter()

    try:
        while True:
            data = rak.receive()

            if data is not None:
                try:
                    if received == 0:
                        start = perf_counter()
                    
                    if not '(Info)' in data['message']:
                        last_index = int(data['message'].split(';')[0])
                        if start_index == 0:
                            start_index = last_index
                        last_index -= start_index - 1
                        strenghts.append(data['signal_strength'])
                        noises.append(data['noise'])
                        received += 1
                        print(f'Received: {data} (lost: {last_index - received}, total: {last_index})')
                    else:
                        print(data['message'])
                except Exception as error:
                    print(error, data)
                

    except KeyboardInterrupt:
        rak.sleep()
        end = perf_counter()
        average_strength = round(sum(strenghts) / received, 2)
        average_noise = round(sum(noises) / received, 2)
        print('\n\n------------RESULT------------')
        print(f'LISTENED FOR: {round(end - start, 2)}s')
        print(f'RECEIVED: {received}')
        print(f'LOST: {last_index - received}')
        print(f'TOTAL: {last_index}')
        print(f'PERCENTILE: {round(received / last_index, 4)*100}%')
        print(f'AVERAGE NOISE: {average_noise}')
        print(f'AVERAGE SIGNAL STRENGTH: {average_strength}')

if mode == 'send':
    rak = Rak4200(serial_port='/dev/ttyS0')
    rak.set_p2p_config(*parameters)
    rak.start('send')
    print('RAK4200 connected, sending test data...')
    index = 1

    while True:
        filler = ''.join(['a'] * 10)
        message = f'{index};{filler}'
        rak.send(message)
        print(f'Sent message {index}')
        index += 1
        sleep(send_timeout)
