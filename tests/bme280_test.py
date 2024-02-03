from bme280 import BME280
from time import sleep

bme280 = BME280()

while True:
    print(f'Pressure: {bme280.get_pressure()}hPa, Temperature: {bme280.get_temperature()}°C, Humidity: {bme280.get_humidity()}%')
    sleep(1)
