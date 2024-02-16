import serial
import time
import pynmea2

def extract_lat_lon(nmea_string):
    lines = nmea_string.split('\n')

    for line in lines:
        if line.startswith("$GPRMC"):
            try:
                msg = pynmea2.parse(line)
                if msg.lat != '':
                    latitude = msg.lat
                    longitude = msg.lon
                    lat_decimal = lat_convert_to_decimal(latitude)
                    lon_decimal = lon_convert_to_decimal(longitude)
                    print("lat,lon:","{},{}".format(lat_decimal, lon_decimal))
            except pynmea2.ParseError as e:
                print('Parse error: {}'.format(e))
def extract_altitude(nmea_string):
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
                
def extract_velocity(nmea_string):
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
                
def lat_convert_to_decimal(coord_str):
    degrees = int(coord_str[:2])
    minutes = float(coord_str[2:])
    decimal_degrees = round(degrees + (minutes / 60),8)
    return decimal_degrees

def lon_convert_to_decimal(coord_str):
    degrees = int(coord_str[:3])
    minutes = float(coord_str[3:])
    decimal_degrees = round(degrees + (minutes / 60),8)
    return decimal_degrees

port = '/dev/ttyACM0'
baud = 9600

serialPort = serial.Serial(port, baudrate=baud, timeout=1)
while True:
    nmea_sentence = serialPort.readline().decode().strip()
    #print(nmea_sentence)
    extract_lat_lon(nmea_sentence)
    extract_altitude(nmea_sentence)
    extract_velocity(nmea_sentence)

