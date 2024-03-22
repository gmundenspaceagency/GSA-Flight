import time
from gps import *

class Gt_u7:
    def __init__(self):
        self.gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

    def get_lat(self):
        nx = self.gpsd.next()
        if nx['class'] == 'TPV':
            latitude = getattr(nx, 'lat', "Unknown")
            return str(latitude)

    def get_lon(self):
        nx = self.gpsd.next()
        if nx['class'] == 'TPV':
            longitude = getattr(nx, 'lon', "Unknown")
            return str(longitude)
        
    def get_altitude(self):
        nx = self.gpsd.next()
        if nx['class'] == 'TPV':
            altitude = getattr(nx, 'alt', "Unknown")
            return str(altitude)

if __name__ == "__main__":
    gt_u7 = Gt_u7()
    while True:
        print(gt_u7.get_lat())
        print(gt_u7.get_lon())
        print(gt_u7.get_altitude())
        time.sleep(1)
