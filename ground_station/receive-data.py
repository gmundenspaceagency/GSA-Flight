from time import sleep
import os
import csv
import datetime
import sys
sys.path.append('..')
from gsa_components.rak4200 import Rak4200

try:
    rak = Rak4200(serial_port='/dev/ttyS0')
    rak.start('receive')
except Exception as error:
    rak.sleep()
    raise Exception('Error while starting RAK: ' + str(error))

print("RAK4200 connnected, receiving data...")

start_time = datetime.datetime.now()
start_time_str = start_time.strftime("%Y-%m-%d_%H-%M-%S")
log_dir = f"/home/gsa202324/GSA-Flight/ground_station/log/groundlog_{start_time_str}/"
os.mkdir(log_dir)
logfile_path = log_dir + "datalog.csv"
infofile_path = log_dir + "info.txt"

with open(logfile_path, "a") as logfile:
    csv_writer = csv.writer(logfile, delimiter = ";")
    csv_writer.writerow(["Timestamp", "Pressure (hPa)", "Temperature (°C)", "Errors", "Mode", "Signal Strength", "Noise", "Message Length"])

while True:
    try:
        response = rak.receive()
        
        try:
            if response is not None:
                strength = response['signal_strength']
                noise = response['noise']
                length = response['message_length']
                message = response['message']

                if not '(Info)' in message:
                    message_vals = message.split(';')
                    [cansat_id, timestamp, pressure, temperature, error_string, mode] = message_vals
                    print(f'CanSat-ID: {cansat_id}, Timestamp: {timestamp}, Pressure: {pressure}hPa, Temperature: {temperature}°C, Errors: {error_string}, Mode: {mode}')

                    with open(logfile_path, "a") as logfile:
                        csv_writer = csv.writer(logfile, delimiter=";")
                        csv_writer.writerow([timestamp, pressure, temperature, error_string, mode, strength, noise, length])
                else:
                    print(message)
                    with open(infofile_path, "a") as logfile:
                        logfile.write(message + '\n')
                
        except Exception as error:
            print(f'Error while processing data {response}: ' + str(error))

    except KeyboardInterrupt:
        print('Program stopped by keyboard interrupt')
        rak.sleep()
        exit()  
    
    except Exception as error:
        print('Error while receiving data: ' + str(error))
        sleep(0.1)
