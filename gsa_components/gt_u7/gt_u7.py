from gps import *
import time

class Gt_u7:

    def get_lat(self, gpsd):
        nx = gpsd.next()
        if nx['class'] == 'TPV':
            latitude = getattr(nx,'lat', "Unknown")
            return str(latitude)

    def get_lon(self, gpsd):
        nx = gpsd.next()
        if nx['class'] == 'TPV':
            longitude = getattr(nx,'lon', "Unknown")
            return str(longitude)
    
    
    def extract_lat(self):
        nmea_sentence = gt_u7.serialPort.readline().decode().strip()
        msg = pynmea2.parse(nmea_sentence)
        return msg.lat

    def extract_lat_lon(self, nmea_string, data_type):
        lines = nmea_string.split('\n')
        for line in lines:
            if line.startswith("$GPRMC"):
                try:
                    msg = pynmea2.parse(line)
                    if msg.lat != '':
                        latitude = msg.lat
                        longitude = msg.lon
                        if data_type == 'decimal':
                            lat_decimal = self.lat_convert_to_decimal(latitude)
                            lon_decimal = self.lon_convert_to_decimal(longitude)
                            print(lat_decimal, lon_decimal, sep=",")
                        if data_type == 'dms':
                            print(latitude, longitude, sep=",")
                except pynmea2.ParseError as e:
                    print('Parse error: {}'.format(e))

    def extract_altitude(self, nmea_string):
        lines = nmea_string.split('\n')

        for line in lines:
            if line.startswith("$GPGGA"):
                try:
                    msg = pynmea2.parse(line)
                    if msg.altitude != '':
                        altitude = msg.altitude
                        print("altitude:", altitude)
                except pynmea2.ParseError as e:
                    print('Parse error: {}'.format(e))
                    
    def extract_velocity(self, nmea_string):
        lines = nmea_string.split('\n')
        for line in lines:
            if line.startswith("$GPRMC"):
                try:
                    msg = pynmea2.parse(line)
                    if msg.spd_over_grnd != '':
                        speed_knots = float(msg.spd_over_grnd)
                        speed_ms = speed_knots * 0.514444  # 1 Knoten = 0,514444 m/s
                        print("velocity:", speed_ms)
                except pynmea2.ParseError as e:
                    print('Parse error: {}'.format(e))

    def extract_satellite_count(self, nmea_string):
        lines = nmea_string.split('\n')
        for line in lines:
            if line.startswith("$GPGGA"):
                try:
                    msg = pynmea2.parse(line)
                    if msg.num_sats != '':
                        num_satellites = int(msg.num_sats)
                        if num_satellites > 0:
                            print("satellite count:", num_satellites)
                        else:
                            print("No satellites found!")
                    else:
                        print("No satellite count data available")
                except pynmea2.ParseError as e:
                    print('Parse error: {}'.format(e))

    def lat_convert_to_decimal(self, coord_str):
        degrees = int(coord_str[:2])
        minutes = float(coord_str[2:])
        decimal_degrees = round(degrees + (minutes / 60), 8)
        return decimal_degrees

    def lon_convert_to_decimal(self, coord_str):
        degrees = int(coord_str[:3])
        minutes = float(coord_str[3:])
        decimal_degrees = round(degrees + (minutes / 60), 8)
        return decimal_degrees

if __name__ == "__main__":
    gt_u7 = Gt_u7()

    while True:
        gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
        print("lat:", gt_u7.get_lat(gpsd))
        print("lon", gt_u7.get_lon(gpsd))
        time.sleep(1.0)
