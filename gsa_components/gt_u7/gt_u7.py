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

if __name__ == "__main__":
    gt_u7 = Gt_u7()
    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
    print(gt_u7.get_lat)
    print(gt_u7.get_lon)
    time.sleep(1)