from time import sleep, perf_counter
import sys
sys.path.append('..')
from gsa_components.rak4200 import Rak4200
import string
import random

mode = 'send' # send or receive

if mode == 'send':
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

if mode == 'receive':
    test = Rak4200(serial_port='/dev/ttyS0')
    test.set_mode('send')
    print('RAK4200 connected, sending test data...')
    index = 0
    s = string.lowercase+string.digits

    while True:
        random_stuff = ''.join(random.sample(s, 10))
        message = f'{index};{random_stuff}'
        rak.send(message)
        index += 1
        print(f'Sent message {index}')
        
