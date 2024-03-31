import serial
import pynmea2
from threading import Thread
import gps
import os
from time import sleep

class Gt_u7:
    def __init__(self, serial_port:str="/dev/ttyACM0", baudrate:int=9600)->None:
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.thread = Thread(target=self.start)
        self.thread.start()

    def start(self)->None:
        self.stop_thread = False
        self.ser = serial.Serial(self.serial_port, self.baudrate)
        self.read_data()

    def read_data(self)->None:
        while not self.stop_thread:
            try:
                data = self.ser.readline().decode().strip()
                if data.startswith("$GPGGA"):
                    msg = pynmea2.parse(data)
                    self.latitude = msg.latitude
                    self.longitude = msg.longitude
                    self.altitude = msg.altitude
            except Exception as e:
                self.latitude = None
                self.longitude = None
                self.altitude = None
                print(f"Error reading GPS data: {e}")
                if not self.stop_thread:
                    try:
                        self.restart()
                    except Exception as e:
                        print(f"Error restarting GPS: {e}")

        sleep(0.3)

    def get_coordinates(self)->list[float]:
        return self.latitude, self.longitude

    def get_altitude(self)->float:
        return self.altitude

    def stop(self)->None:
        self.stop_thread = True
        sleep(0.5)
        self.ser.close()

    def restart(self)->None:
        self.stop()
        sleep(0.5)
        self.start()

if __name__ == "__main__":
    gps = Gt_u7()

    while True:
        print(f"lon:{gps.get_coordinates()[1]}")
        print(f"lat:{gps.get_coordinates()[0]}")
        print(f"alt:{gps.get_altitude()}")
        sleep(1)
