from gps import *
import time

class Gt_u7:
    def get_lat(self, nx):
        nx = gpsd.next()
        if nx['class'] == 'TPV':
            latitude = getattr(nx, 'lat', "Unknown")
            return str(latitude)

    def get_lon(self, nx):
        nx = gpsd.next()
        if nx['class'] == 'TPV':
            longitude = getattr(nx, 'lon', "Unknown")
            return str(longitude)
        
    def get_altitude(self, nx):
        nx = gpsd.next()
        if nx['class'] == 'TPV':
            altitude = getattr(nx, 'alt', "Unknown")
            return str(altitude)

if __name__ == "__main__":
    gt_u7 = Gt_u7()
    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
    while True:
        print(gt_u7.get_lat(gpsd))
        print(gt_u7.get_lon(gpsd))
        print(gt_u7.get_altitude(gpsd))
        time.sleep(1)
