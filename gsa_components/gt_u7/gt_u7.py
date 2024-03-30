import serial
import pynmea2
from threading import Thread

class Gt_u7:
    def __init__(self, serial_port="/dev/ttyACM0", baudrate=9600):
        self.ser = serial.Serial(serial_port, baudrate)
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.stop_thread = False
        self.thread = Thread(target=self.read_data_thread)
        self.thread.start()

    def read_data(self):
        while True:
            data = self.ser.readline().decode().strip()
            if data.startswith("$GPGGA"):
                msg = pynmea2.parse(data)
                self.latitude = msg.latitude
                self.longitude = msg.longitude
                self.altitude = msg.altitude

    def read_data_thread(self):
        self.read_data()

    def get_coordinates(self):
        return self.latitude, self.longitude

    def get_altitude(self):
        return self.altitude

    def stop(self):
        self.stop_thread = True
        self.thread.join()

if __name__ == "__main__":
    gps = Gt_u7()

    while True:
        latitude, longitude = gps.get_coordinates()
        altitude = gps.get_altitude()

        print(f"lon:{longitude}")
        print(f"lat:{latitude}")
        print(f"alt:{altitude}")
