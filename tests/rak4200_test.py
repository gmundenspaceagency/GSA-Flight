from time import sleep, perf_counter
import sys
sys.path.append('..')
from gsa_components.rak4200 import Rak4200
import random
import platform

mode = 'send' if platform.machine() == 'armv6l' else 'receive'

if mode == 'receive':
    rak = Rak4200(serial_port='/dev/ttyUSB0')
    rak.start('receive')
    print('RAK4200 connected, receiving messages...')
    received = last_index = 0
    strenghts = noises = []
    start = perf_counter()

    try:
        while True:
            data = rak.receive()

            if data is not None:
                if received == 0:
                    start = perf_counter()
                
                last_index = int(data['message'].split(';')[0])
                strenghts.append(data['signal_strength'])
                noises.append(data['noise'])
                received += 1

                print(f'Received: {data} (lost: {last_index - received}, total: {last_index})')
    except KeyboardInterrupt:
        end = perf_counter()
        average_strength = round(sum(strenghts) / received, 2)
        average_noise = round(sum(noises) / received, 2)
        print('------------RESULT------------')
        print(f'LISTENED FOR: {end - start}s')
        print(f'RECEIVED: {received}')
        print(f'LOST: {last_index - received}')
        print(f'TOTAL: {last_index}')
        print(f'PERCENTILE: {round(received / last_index, 2)}')
        print(f'AVERAGE NOISE: {average_noise}')
        print(f'AVERAGE SIGNAL STRENGTH: {average_strength}')

if mode == 'send':
    rak = Rak4200(serial_port='/dev/ttyS0')
    rak.set_mode('send')
    print('RAK4200 connected, sending test data...')
    index = 0
    send_timeout = 0.5

    while True:
        filler = ''.join(['a'] * 10)
        message = f'{index};{filler}'
        rak.send(message)
        index += 1
        print(f'Sent message {index}')
        sleep(send_timeout)
