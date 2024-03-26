import time
from gps import gps, WATCH_ENABLE

class Gt_u7:
    def __init__(self):
        self.session = gps(mode=WATCH_ENABLE)

    def get_data(self):
        for report in self.session:
            if report['class'] == 'TPV':
                latitude = getattr(report, 'lat', "Unknown")
                longitude = getattr(report, 'lon', "Unknown")
                altitude = getattr(report, 'alt', "Unknown")
                return str(latitude), str(longitude), str(altitude)

if __name__ == "__main__":
    gt_u7 = Gt_u7()
    while True:
        lat, lon, alt = gt_u7.get_data()
        print(lat)
        print(lon)
        print(alt)
        time.sleep(1)
