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
        
    def getPositionData(self, gpsd):
        nx = gpsd.next()
        if nx['class'] == 'TPV':
            latitude = getattr(nx,'lat', "Unknown")
            longitude = getattr(nx,'lon', "Unknown")
            print ("Your position: lon = " + str(longitude) + ", lat = " + str(latitude))
    
    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

    while True:
        print(get_lon)
        getPositionData(gpsd)
        time.sleep(1.0)

if __name__ == "__main__":
    gt_u7 = Gt_u7()

    while True:
        gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
        print("lat:", gt_u7.get_lat(gpsd))
        print("lon", gt_u7.get_lon(gpsd))
        gt_u7.getPositionData(gpsd)
        time.sleep(1.0)
