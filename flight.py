import time
from time import sleep
import datetime
from threading import Thread
from gpiozero import LED, Button
from bh1750 import BH1750
import Adafruit_ADS1x15
from bme import BME
from gyroscope import GYRO
import RPi.GPIO as GPIO
from typing import Optional
from motor import MOTOR
from rak4200 import RAK4200
from picamera2.encoders import H264Encoder
from picamera2 import Picamera2, Preview

cansat_id = '69xd'
pi_state = 'initializing'
print('Pi is initializing...')

# is True when a second thread with the blink_onboard function is running
blinking = False

# initialize and test onboard led
try:
    onboard_led = LED(23)

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
    Thread(target=blink_onboard, args=(0.5, 'initializing')).start()
except Exception as error:
    onboard_led = None
    print('Warning: Onboard LED could not be initialized: ' + str(error))

# initialize sensors
def initialize_light1()->Optional[BH1750]:
    try:
        light1 = BH1750()
        light1.luminance(BH1750.ONCE_HIRES_1)
    except Exception as error:
        light1 = None
        print('Problem with light sensor 1: ' + str(error))
    
    return light1

def initialize_light2()->Optional[BH1750]:   
    try:
        GPIO.output(power_light2, 1)
        GPIO.output(power_light3, 0)
        
        # wait until both pins have the desired states
        while not GPIO.input(power_light2) or GPIO.input(power_light3):
            pass
        
        light2 = BH1750(1, 0x5c)
        light2.luminance(BH1750.ONCE_HIRES_1)
    except Exception as error:
        light2 = None
        print('Problem with light sensor 2: ' + str(error))
    
    return light2

def initialize_light3()->Optional[BH1750]:
    try:
        GPIO.output(power_light2, 0)
        GPIO.output(power_light3, 1)
        
        # wait until both pins have the desired states
        while GPIO.input(power_light2) or not GPIO.input(power_light3):
            pass
        
        light3 = BH1750(1, 0x5c)
        light3.luminance(BH1750.ONCE_HIRES_1)
    except Exception as error:
        light3 = None
        print('Problem with light sensor 3: ' + str(error))
    
    return light3

def initialize_adc()->Optional[Adafruit_ADS1x15.ADS1115]:
    try:
        adc = Adafruit_ADS1x15.ADS1115()
        adc.read_adc(1, gain=1)
    except Exception as error:
        adc = None
        print('Problem with A/D-Converter: ' + str(error))
    
    return adc

def initialize_bme()->Optional[BME]:
    try:
        bme = BME()
        bme.read_data()
    except Exception as error:
        bme = None
        print('Problem with BME-Sensor: ' + str(error))
    
    return bme
 
def initialize_gyro()->Optional[GYRO]:
    try:
        gyro = GYRO()
        gyro.get_scaled_acceleration()
    except Exception as error:
        gyro = None
        print('Problem with gyroscope: ' + str(error))
    
    return gyro

def initialize_motor()->Optional[MOTOR]:
    try:
        motor = MOTOR(19, 26, 20, 21)
        motor.move_angle(120)
        # sleep(0.5)
        # motor.set_angle(270)
        # sleep(0.5)
        # motor.set_angle(0)
    except Exception as error:
        motor = None
        print('Problem with motor: ' + str(error))
    
    return motor

"""
Günther is the TRANSCEIVER!!!
"""

def initialize_guenther()->Optional[RAK4200]:
    try:
        guenther = RAK4200()
        guenther.start()
    except Exception as error:
        guenther = None
        print('Problem with guenther: ' + str(error))
    
    return guenther
        
try:
    GPIO.setmode(GPIO.BCM)
    power_light2 = 17
    power_light3 = 27
    GPIO.setup(17, GPIO.OUT, initial=0)
    GPIO.setup(27, GPIO.OUT, initial=0)
    power_button = Button(22, pull_up=True)
    
    # every sensor is in a try-block so the program will still work when one of the sensors fails on launch
    light1 = initialize_light1()
    light2 = initialize_light2()
    light3 = initialize_light3()
    adc = initialize_adc()
    bme = initialize_bme()
    gyro = initialize_gyro()
    motor = initialize_motor()
    guenther = initialize_guenther()
    
    try:
        picam2 = Picamera2()
        video_config = picam2.create_video_configuration(main={"size": (1920, 1080)})
        picam2.configure(video_config)
        encoder = H264Encoder(bitrate=1000000)
        output = '/home/gsa202324/Desktop/test.h264'
    except Exception as error:
     print('Problem with camera: ' + str(error))
           
except KeyboardInterrupt:
    print('Initializing aborted by keyboard interrupt')
    GPIO.cleanup()
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
    GPIO.cleanup()
    exit()
"""
luminance1 = luminance2 = luminance3 = None

def rotation_mechanism() -> None:
    global luminance1, luminance2, luminance3
    
    while pi_state == 'running' or pi_state == 'shutting down':
        try:
            GPIO.setmode(GPIO.BCM)
            power_light2 = 17
            power_light3 = 27
            GPIO.setup(17, GPIO.OUT, initial=0)
            GPIO.setup(27, GPIO.OUT, initial=0)
            GPIO.output(power_light2, 1)
            GPIO.output(power_light3, 0)
                
            try:
                luminance1 = round(light1.luminance(BH1750.ONCE_HIRES_1), 4)
            except Exception as error:
                # try to contact sensor again
                light1 = initialize_light1()

            try: 
                # wait until both pins have the desired states
                while not GPIO.input(power_light2) or GPIO.input(power_light3):
                    pass
                
                luminance2 = round(light2.luminance(BH1750.ONCE_HIRES_1), 4)
            except Exception as error:
                # try to contact sensor again
                light2 = initialize_light2()

            try:
                GPIO.output(power_light2, 0)
                GPIO.output(power_light3, 1)
                
                # wait until both pins have the desired states
                while GPIO.input(power_light2) or not GPIO.input(power_light3):
                    pass
                
                luminance3 = round(light3.luminance(BH1750.ONCE_HIRES_1), 4)
            except Exception as error:
                # try to contact sensor again
                light3 = initialize_light3()
            
            if luminance1 is not None and luminance2 is not None and luminance3 is not None:
                luminances = [luminance1, luminance2, luminance3]
                angle_fraction = 360 / len(luminances)
                highest = max(luminances)
                index = luminances.index(highest)
                right = luminances[index + 1] if index + 1 < len(luminances) else luminances[0]
                left = luminances[index - 1] if index > 0 else luminances[-1]
                
                if right > left:
                    goal_angle = round(index / len(luminances) * (360 - angle_fraction) + angle_fraction * right / max(highest, 1))
                else:
                    goal_angle = round(index / len(luminances) * (360 - angle_fraction) - angle_fraction * left / max(highest, 1))
                
                motor.set_angle(goal_angle)
        except KeyboardInterrupt as error:
            print('Error in rotation mechanism: ' + str(error))

def main()->None:
    global pi_state, adc, bme, gyro, guenther

    start_time = datetime.datetime.now()
    start_perf = round(time.perf_counter() * 1000)
    # TODO: realistische iteration time
    max_iteration_time = 2000
    timestamps = []
    pressures = []
    temperatures = []
    humidities = []
    altitudes = []
    vertical_speeds = []
    vertical_accelerations = []
    rotationrates = []
    gyro_accelerations = []
    rotationangles = []
    
    Thread(target=rotation_mechanism, args=()).start()
    sleep(1)
    
    # TODO: vor dem flug die onboard led sicher machen (falls verbindung weg) oder ganz removen
    while True:
        timestamp = round(time.perf_counter() * 1000 - start_perf)
        timestamps.append(timestamp)

        if onboard_led is not None:
            onboard_led.off()
        
        if len(timestamps) > 1:
            time_difference = timestamp - timestamps[-2]
            
            if time_difference > max_iteration_time:
                print(f'Warning: Single iteration took more than {max_iteration_time}ms (took {time_difference}ms)')

        print(f'Luminance at sensors (lux): {luminance1} {luminance2} {luminance3}')

        try:
            adc_value = adc.read_adc(1, gain=1)
            solar_voltage = round(4.096 / 32767 * adc_value, 3)
            print(f'Solar panel voltage: {solar_voltage}V')
        except Exception as error:
            # try to contact sensor again
            adc = initialize_adc()
        
        # X means they were not set yet
        pressure = 'X'
        temperature = 'X'
        
        try:
            data = bme.read_data()
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
                avg_over = 5

                avg_vertical_speed = round(sum(vertical_speeds[max(-len(timestamps) + 1, -avg_over):]) / min(len(timestamps) - 1, avg_over), 2)
                vertical_speed = round((altitude - altitudes[-2]) / time_difference * 1000, 2)
                vertical_speeds.append(vertical_speed)

                print(f'Altitude: {altitude}m, Speed: {vertical_speed}m/s, Average speed: {avg_vertical_speed}m/s')

            # acceleration can only be calculated after 3 height measures
            if len(timestamps) > 2:
                avg_vertical_acceleration = round(sum(vertical_accelerations[max(-len(timestamps) + 1, -avg_over):]) / min(len(timestamps) - 1, avg_over), 2)

                vertical_acceleration = round((vertical_speed - vertical_speeds[-2]) / time_difference, 3)
                vertical_accelerations.append(vertical_acceleration)

                print(f'Acceleration: {vertical_acceleration}m/s^2, Average acceleration: {avg_vertical_acceleration}m/s^2')
        except Exception as error:
            # try to contact sensor again
            bme = initialize_bme()
    
        try:
            gyro_data = gyro.get_scaled_gyroscope()
            rotationrates.append(gyro_data)
            rotationrate_x, rotationrate_y, rotationrate_z = gyro_data
            acceleration = gyro.get_scaled_acceleration()
            gyro_accelerations.append(acceleration)
            acceleration_x, acceleration_y, acceleration_z = acceleration
            rotation = gyro.get_rotation(*acceleration)
            rotationangles.append(rotation)
            rotation_x, rotation_y = rotation
            
            print(f'Rate of rotation (°/s): {rotationrate_x}x {rotationrate_y}y {rotationrate_z}z')
            print(f'Static Acceleration (g): {acceleration_x}x {acceleration_y}y {acceleration_z}z')
            print(f'Angle of rotation (°): {rotation_x}x {rotation_y}y')
        except Exception as error:
            # try to contact sensor again
            gyro = initialize_gyro()
        
        try:
            guenther.send(f'{cansat_id}-{timestamp}-{rotation_x}-{rotation_y}')
        except Exception as error:
            # try to contact transceiver again
            guenther = initialize_guenther()
        
        if onboard_led is not None:
            onboard_led.on()

        # check for turn off
        if power_button.is_pressed:
            pi_state = 'shutting down'
            
            while blinking:
                pass
            
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
    if motor is not None:
        motor.set_angle(0)
    
    GPIO.cleanup()
    pi_state = 'off'
    currentTime = datetime.now()
    currentTimeStr = currentTime.strftime("%H:%M:%S")
    guenther.send('(Info) CanSat program finished at: ' + currentTimeStr) 
    print('bye bye')
