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
        
    def get_coordinates(self):
        msg = self.read_data()
        if msg:
            latitude = msg.lat
            longitude = msg.lon
            return latitude, longitude
        else:
            return None, None
        
    def get_altitude(self):
        msg = self.read_data()
        if msg:
            altitude = msg.altitude
            return altitude
        else:
            return None

if __name__ == "__main__":
    gps = Gt_u7()

    while True:
        latitude, longitude = gps.get_coordinates()
        altitude = gps.get_altitude()

        if latitude is not None and longitude is not None:
            print("Latitude:", latitude)
            print("Longitude:", longitude)
        else:
            print("Keine GPS-Daten verfügbar.")

        if altitude is not None:
            print("Höhe:", altitude)
        else:
            print("Keine Höhendaten verfügbar.")