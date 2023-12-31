import time
from time import sleep
import datetime
from threading import Thread
from smbus import SMBus
from gpiozero import LED, Button
from bh1750 import BH1750
import Adafruit_ADS1x15
from bme import BME

pi_state = 'initializing'
print('Pi is initializing...')

# is True when a second thread with the blink_onboard function is running
blinking = False

# initialize and test onboard led
try:
    onboard_led = LED(25)

    # controls blinking of pi led to indicate different states of pi
    def blink_onboard(timeout: float, state: str) -> None:
        global blinking
        blinking = True

        while pi_state == state:
            onboard_led.on()
            sleep(timeout)
            onboard_led.off()
            sleep(timeout)

        blinking = False

    # fast blinking to show pi is initializing
    Thread(target=blink_onboard, args=(0.05, 'initialize')).start()

except Exception as error:
    onboard_led = None
    print('Warning: Onboard LED could not be initialized: ' + str(error))

# initialize sensors
try:
    # every sensor is in a try-block so the program will still work when one of the sensors fails on launch
    try:
        light1 = BH1750()
        light1.luminance(BH1750.ONCE_HIRES_1)
    except Exception as error:
        light1 = None
        print('Warning: Error while testing light sensor 1: ' + str(error))

    power_button = Button(5, pull_up=True)

    try:
        adc = Adafruit_ADS1x15.ADS1115()
        adc.read_adc(1, gain=1)
    
    except Exception as error:
        adc = None
        print('Warning: Error while testing A/D-Converter: ' + str(error))

    try:
        pth = BME()
        pth.read_data()
    except Exception as error:
        pth = None
        print('Warning: Error while testing PTH-Sensor: ' + str(error))

except KeyboardInterrupt:
    print('Initializing aborted by keyboard interrupt')

    if onboard_led is not None:
        onboard_led.off()

    exit()
"""
try:
    pi_state = 'ready'
    print('Pi is ready, hold start button to start program')

    # wait until the second thread with the blink_onboard function has stopped
    while blinking:
        pass

    # blinking to show the pi is ready
    Thread(target=blink_onboard, args=(0.1, 'ready')).start()

    # wait for 1s button press
    while True:
        if power_button.is_pressed:
            pi_state = 'starting'

            while blinking:
                pass

            # fast flashing telling you should keep holding the button
            Thread(target=blink_onboard, args=(0.05, 'starting')).start()
            sleep(1)

            if power_button.is_pressed:
                break
            else:
                pi_state = 'ready'

                while blinking:
                    pass

                Thread(target=blink_onboard, args=(0.5, 'ready')).start()

except KeyboardInterrupt:
    print('Program stopped in ready state by keyboard interrupt')

    if onboard_led is not None:
        onboard_led.off()

    exit()
"""
def main() -> None:
    global pi_state

    start_time = datetime.datetime.now()
    start_perf = time.perf_counter() * 1000
    timestamps = []
    pressures = []
    temperatures = []
    humidities = []
    altitudes = []
    vertical_speeds = []
    vertical_accelerations = []
    
    # TODO: zu allen sensor readings ein except hinzufügen falls was ausfällt
    while True:
        timestamp = time.perf_counter() * 1000 - start_perf
        timestamps.append(timestamp)

        if onboard_led is not None:
            onboard_led.off()

        if light1 is not None:
            luminance1 = round(light1.luminance(BH1750.ONCE_HIRES_1), 4)
            print(f'Luminance at Sensor 1: {luminance1}lux')

        if adc is not None:
            adc_value = adc.read_adc(1, gain=1)
            solar_voltage = round(4.096 / 32767 * adc_value, 3)
            print(f'Solar panel voltage: {solar_voltage}V')

        if pth is not None:
            data = pth.read_data()
            pressure = round(float(data.pressure), 2)
            pressures.append(pressure)
            temperature = round(float(data.temperature), 2)
            temperatures.append(temperature)
            humidity = round(float(data.humidity) / 100, 2)
            humidities.append(humidity)
            altitude = round(44331.5 - 4946.62 * (pressure * 100) ** 0.190263, 2)
            altitudes.append(altitude)

            print(f'Pressure: {pressure}hPa, temperature: {temperature}°C, humidity: {humidity * 100}%')

            # speed can only be calculated after 2 height measures
            if len(timestamps) > 1:
                time_difference = timestamp - timestamps[-2]
                avg_over = 5

                avg_vertical_speed = round(sum([
                    (alt - altitudes[i - 1]) / (timestamps[i] - timestamps[i - 1]) * 1000
                    for i, alt
                    in enumerate(altitudes[max(-len(timestamps) + 1, -avg_over - 1):-1])
                ]) / min(len(timestamps), avg_over), 2)

                vertical_speed = round((altitude - altitudes[-2]) / time_difference * 1000, 2)
                vertical_speeds.append(vertical_speed)

                print(f'Altitude: {altitude}m, Speed: {vertical_speed}m/s, Average speed: {avg_vertical_speed}m/s')

            # acceleration can only be calculated after 3 height measures
            if len(timestamps) > 2:
                avg_vertical_acceleration = round(sum([
                    speed - vertical_speeds[i - 1]
                    for i, speed
                    in enumerate(vertical_speeds[max(-len(timestamps) + 2, -avg_over - 1):-1])
                ]) / min(len(timestamps), avg_over), 3)

                vertical_acceleration = round((vertical_speed - vertical_speeds[-2]) / time_difference, 3)
                vertical_accelerations.append(vertical_acceleration)

                print(f'Acceleration: {vertical_acceleration}m/s^2, Average acceleration: {avg_vertical_acceleration}m/s^2')

        if onboard_led is not None:
            onboard_led.on()

        # check for turn off
        if power_button.is_pressed:
            pi_state = 'shutting down'

            # fast flashing telling pi is about to shut down
            Thread(target=blink_onboard, args=(0.05, 'shutting down')).start()
            sleep(1)

            if power_button.is_pressed:
                print('Program switched off with power button')

                if onboard_led is not None:
                    onboard_led.on()

                exit()
            else:
                pi_state = 'running'

                while blinking:
                    pass

                if onboard_led is not None:
                    onboard_led.on()

        sleep(1)

try:
    pi_state = 'running'
    print('Program is running')

    while blinking:
        pass

    if onboard_led is not None:
        onboard_led.on()

    # wait until power button is let go
    while power_button.is_pressed:
        pass

    main()

except KeyboardInterrupt:
    print('Running program stopped by keyboard interrupt')

    if onboard_led is not None:
        onboard_led.off()

finally:
    pi_state = 'off'

    if onboard_led is not None:
        onboard_led.off()

    print('bye bye')
