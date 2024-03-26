import time
from gps import *

class Gt_u7:
    def __init__(self):
        self.gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

    def get_data(self):
        nx = self.gpsd.next()
        if nx['class'] == 'TPV':
            latitude = nx.get('lat', "Unknown")
            longitude = nx.get('lon', "Unknown")
            altitude = nx.get('alt', "Unknown")
            return str(latitude), str(longitude), str(altitude)

if __name__ == "__main__":
    gt_u7 = Gt_u7()
    while True:
        lat, lon, alt = gt_u7.get_data()
        print(lat)
        print(lon)
        print(alt)
        time.sleep(1)
