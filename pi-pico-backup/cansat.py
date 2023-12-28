import sys
import _thread
import bme280
import time
from time import sleep
from machine import SoftI2C, I2C, Pin
from bh1750 import BH1750
from ads111x import ADS111X 

pi_state = 'initializing'
print('Pi is initializing...')

# is True when a second thread with the blink_onboard function is running
blinking = False

# initialize and test onboard led
try:
    onboard_led = Pin(25, Pin.OUT, value=0)

    # controls blinking of pi led to indicate different states of pi
    def blink_onboard(timeout:float, state:str)->None:
        if onboard_led is None:
            return None
    
        global blinking, time
        blinking = True
        
        while pi_state == state:
            onboard_led.on()
            sleep(timeout)
            onboard_led.off()
            sleep(timeout)
            
        blinking = False

    # fast blinking to show pi is initializing
    _thread.start_new_thread(blink_onboard, (0.05, 'initialize'))

except Exception as error:
    onboard_led = None
    print('Warning: Onboard LED could not be initialized: ' + str(error))

# initialize sensors
try:
    # every sensor is in a try-block so the program will still work when one of the sensors fails on launch
    try:
        light1 = BH1750(I2C(0, sda=Pin(20), scl=Pin(21)))
        light1.luminance(BH1750.ONCE_HIRES_1)
    except Exception as error:
        light1 = None
        print('Warning: Error while testing light sensor 1: ' + str(error))

    power_button = Pin(5, Pin.IN, Pin.PULL_UP)
    
    try:
        atod = ADS111X(15, 14)
        atod.read_value()
    except Exception as error:
        atod = None
        print('Warning: Error while testing A/D-Converter: ' + str(error))
    
    try:
        # using SoftI2C due to a weird problem with the i2c communication of this specific bme280 model (https://github.com/orgs/micropython/discussions/10506)
        pth = bme280.BME280(i2c=SoftI2C(sda=Pin(0), scl=Pin(1), freq=400000))
        pth.value
    except Exception as error:
        pth = None
        print('Warning: Error while testing PTH-Sensor: ' + str(error))

except KeyboardInterrupt:
    print('Initializing aborted by keyboard interrupt')

    if onboard_led is not None:
        onboard_led.off()
        
    sys.exit()

try:
    pi_state = 'ready'
    print('Pi is ready, hold start button to start program')

    # wait until the second thread with the blink_onboard function has stopped
    while blinking: 
        pass

    # blinking to show the pi is ready
    _thread.start_new_thread(blink_onboard, (0.1, 'ready'))

    # wait for 1s button press
    while True:
        if power_button.value() == 0:
            pi_state = 'starting'

            while blinking: 
                pass

            # fast flashing telling you should keep holding the button
            _thread.start_new_thread(blink_onboard, (0.05, 'starting'))
            sleep(1)

            if power_button.value() == 0:
                break
            else:
                pi_state = 'ready'

                while blinking: 
                    pass
                
                _thread.start_new_thread(blink_onboard, (0.5, 'ready'))
    
except KeyboardInterrupt:
    print('Program stopped in ready state by keyboard interrupt')

    if onboard_led is not None:
        onboard_led.off()

    sys.exit()

def main()->None:
    global pi_state
    
    start_time = time.localtime()
    timestamps = []
    pressures = []
    temperatures = []
    humidities = []
    altitudes = []
    vertical_speeds = []
    vertical_accelerations = []
    
    while True:
        timestamp = time.ticks_ms()
        timestamps.append(timestamp)
        
        if onboard_led is not None:
            onboard_led.off()
        
        if light1 is not None:
            luminance1 = light1.luminance(BH1750.ONCE_HIRES_1)
            print(f'Luminance at Sensor 1: {luminance1}lux')
        
        if atod is not None:
            solar_voltage = round(atod.val_to_voltage(atod.read_value()), 3)
            print(f'Solar panel voltage: {solar_voltage}V')
        
        if pth is not None:
            pressure = float(pth.values[1].replace('hPa', ''))
            pressures.append(pressure)
            temperature = float(pth.values[0].replace('C', ''))
            temperatures.append(temperature)
            humidity = float(pth.values[2].replace('%', '')) / 100
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
                    in enumerate(altitudes[max(-len(timestamps) + 1,-avg_over - 1):-1])
                ]) / min(len(timestamps), avg_over), 2)
                
                vertical_speed = round((altitude - altitudes[-2]) / time_difference * 1000, 2)
                vertical_speeds.append(vertical_speed)
                                
                print(f'Altitude: {altitude}m, Speed: {vertical_speed}m/s, Average speed: {avg_vertical_speed}m/s')
            
            # acceleration can only be calculated after 3 height measures
            if len(timestamps) > 2:
                avg_vertical_acceleration = round(sum([
                    speed - vertical_speeds[i - 1]
                    for i, speed
                    in enumerate(vertical_speeds[max(-len(timestamps) + 2,-avg_over - 1):-1])
                ]) / min(len(timestamps), avg_over), 3)
                
                vertical_acceleration = round((vertical_speed - vertical_speeds[-2]) / time_difference, 3)
                vertical_accelerations.append(vertical_acceleration)
                
                print(f'Acceleration: {vertical_acceleration}m/s^2, Average acceleration: {avg_vertical_acceleration}m/s^2')

                    
        if onboard_led is not None:
            onboard_led.on()
        
        # check for turn off
        if power_button.value() == 0:
            pi_state = 'shutting down'

            # fast flashing telling pi is about to shut down
            _thread.start_new_thread(blink_onboard, (0.05, 'shutting down'))
            sleep(1)

            if power_button.value() == 0:
                print('Program switched off with power button')

                if onboard_led is not None:
                    onboard_led.on()
                
                sys.exit()
            else:
                pi_state = 'running'

                while blinking: 
                    pass

                if onboard_led is not None:
                    onboard_led.on()
        
        sleep(0.5)
                
try:
    pi_state = 'running'
    print('Program is running')

    while blinking: 
        pass

    if onboard_led is not None:
        onboard_led.on()
    
    # wait until power button is let go
    while power_button.value() == 0:
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
