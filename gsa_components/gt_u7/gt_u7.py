import serial
import pynmea2

class Gt_u7:

    def __init__(self, serial_port="/dev/ttyACM0", baudrate=9600):
        self.ser = serial.Serial(serial_port, baudrate)

    def read_data(self):
        data = self.ser.readline().decode().strip()
        if data.startswith("$GPGGA"):
            msg = pynmea2.parse(data)
            return msg

    def get_coordinates(self, msg):
        if msg:
            latitude = msg.latitude
            longitude = msg.longitude
            return latitude, longitude
        else:
            return None, None

    def get_altitude(self, msg):
            if msg:
                altitude = msg.altitude
                return altitude
            else:
                return None

if __name__ == "__main__":
    gps = Gt_u7()

    while True:
        msg = gps.read_data()
        latitude, longitude = gps.get_coordinates(msg)
        altitude = gps.get_altitude(msg)

        print(f"lon:{longitude}")
        print(f"lat:{latitude}")
        print(f"alt:{altitude}")

