import time
import smbus
import datetime
import RPi.GPIO as GPIO

from typing import Optional
from threading import Thread
from gpiozero import LED, Button
from time import sleep, perf_counter

# pimoroni-bme280 1.0.0
from bme280 import BME280
from Adafruit_ADS1x15 import ADS1115
from picamera2.encoders import H264Encoder
from picamera2 import Picamera2, Preview
from gsa_components.mpu6050.mpu6050 import Mpu6050
from gsa_components.bh1750.bh1750 import Bh1750
from gsa_components.motor import Motor
from gsa_components.rak4200 import Rak4200
from tests import CircularPIDController

"""
Allgemeine ToDos:

- Multiplexer class
- GPS Daten lesen, an die Bodenstation senden ob wir FIX haben oder nicht
    - GPS Höhendaten mit dem bme280280 Daten abgleichen
- Temperaturdaten von GYRO und bme280280 abgleichen
- Kameraaufnahmen machen
- Detailliertes Flight-Log erstellen (Flight_19d-10m-2024y_17h-35m-01s.json)
    - Für jeden timestap ALLE DATEN die wir überhaupt nur bekommen können speichern
- Error handling
    - Wenn ein Error zum ersten Mal auftritt: speichern wann er aufgetreten ist, den Fehlertext (str(error)), und wenn er wieder weggeht speicher wann er weggegangen ist
    - Trotzdem noch alle Fehler printen
    - In einer JSON Datei allen unseren Fehlern error codes zuordnen (eg. 01 -> Fehler mit Lichtsensor1, 08 -> Fehler mit bme280280)
        - In einem Array aktuell aktive Fehler speichern und dieses Array wenn möglich zur Bodenstation schicken
- Bodenstation und Transceiver
    - Empfangene Daten in einem bodenstation-log.json speichern
    - Kleine UI mit empfangenen Daten, mögliche Fehler Codes darstellen
- Arbeit mit Daten nach Flug
    - Wenn schon Flight-Log erstellt, probieren die Daten in coolen Diagrammen darzustellen
- Script sofort nach Start von Pi ausführen
"""

CANSAT_ID = '69xd'
BUS = smbus.SMBus(1)
channel_array=[0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000,0b10000000]
pi_state = 'initializing'
print('Pi is initializing...')

# is True when a second thread with the blink_onboard function is running
blinking = False

# initialize and test onboard led
try:
    status_led = LED(23)

    # controls blinking of pi led to indicate different states of pi
    def blink_onboard(timeout: float, state: str) -> None:
        global blinking
        blinking = True

        while pi_state == state:
            status_led.on()
            sleep(timeout)
            status_led.off()
            sleep(timeout)

        blinking = False

    # blinking to show pi is initializing
    Thread(target=blink_onboard, args=(0.5, 'initializing')).start()
except Exception as error:
    print('Warning: Onboard LED could not be initialized: ' + str(error))

# initialize sensors
def initialize_light1()->Optional[Bh1750]:
    try:
        light1 = Bh1750()
        light1.luminance(Bh1750.ONCE_LOWRES)
    except Exception as error:
        light1 = None
        print('Problem with light sensor 1: ' + str(error))
    
    return light1

def initialize_light2n3()->Optional[Bh1750]:   
    try:
        BUS.write_byte(0x70, channel_array[6])
        light2 = Bh1750(1, 0x5c)
        light2.luminance(Bh1750.ONCE_LOWRES)
    except Exception as error:
        light2 = None
        print('Problem with light sensor 2+3 or Multiplexer: ' + str(error))
    
    return light2

def initialize_ads1115()->Optional[ADS1115]:
    try:
        ads1115 = ADS1115()
        ads1115.read_adc(0, gain=1)
    except Exception as error:
        ads1115 = None
        print('Problem with A/D-Converter: ' + str(error))
    
    return ads1115

def initialize_bme280()->Optional[BME280]:
    try:
        bme280 = BME280()
        bme280.get_temperature()
    except Exception as error:
        bme280 = None
        print('Problem with BME280-Sensor: ' + str(error))
    
    return bme280
 
def initialize_mpu6050()->Optional[Mpu6050]:
    try:
        mpu6050 = Mpu6050()
        mpu6050.get_scaled_acceleration()
    except Exception as error:
        mpu6050 = None
        print('Problem with gyroscope: ' + str(error))
    
    return mpu6050

def initialize_motor()->Optional[Motor]:
    try:
        motor = Motor(26,19,13,6,200,0.002)
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

def initialize_guenther()->Optional[Rak4200]:
    try:
        guenther = Rak4200()
        guenther.start('send')
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
    light2n3 = initialize_light2n3()
    ads1115 = initialize_ads1115()
    bme280 = initialize_bme280()
    mpu6050 = initialize_mpu6050()
    motor = initialize_motor()
    guenther = initialize_guenther()
    
    try:
        picam2 = Picamera2()
        video_config = picam2.create_video_configuration(main={"size": (1920, 1080)})
        picam2.configure(video_config)
        encoder = H264Encoder(bitrate=1000000)
        output = '/home/gsa202324/Desktop/test.h264'
    except Exception as error:
        print('Problem with camera:' + str(error))
           
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
    
    current_angle = 0

    while pi_state == 'running' or pi_state == 'shutting down':            
        try:  
            try:
                luminance1 = round(light1.luminance(Bh1750.ONCE_LOWRES), 4)
            except Exception as error:
                # try to contact sensor again
                light1 = initialize_light1()
            try:
                BUS.write_byte(0x70, channel_array[7])
                luminance2 = round(light2n3.luminance(Bh1750.ONCE_LOWRES), 4)
            except Exception as error:
                # try to contact sensor again
                light2 = initialize_light2n3()
                
            try:
                BUS.write_byte(0x70, channel_array[6])
                luminance3 = round(light2n3.luminance(Bh1750.ONCE_LOWRES), 4)
            except Exception as error:
                # try to contact sensor again
                light3 = initialize_light2n3()
            
            if luminance1 is not None and luminance2 is not None and luminance3 is not None:
                luminances = [luminance1, luminance3, luminance2]
                angle_fraction = 360 / len(luminances)
                highest = max(luminances)
                index = luminances.index(highest)
                right = luminances[index + 1] if index + 1 < len(luminances) else luminances[0]
                left = luminances[index - 1] if index > 0 else luminances[-1]
                
                if right > left:
                    goal_angle = round(index / (len(luminances) - 1) * (360 - angle_fraction) + angle_fraction * right / max(highest, 1))
                else:
                    goal_angle = round(index / (len(luminances) - 1) * (360 - angle_fraction) - angle_fraction * left / max(highest, 1))
                
                if goal_angle < 0:
                    goal_angle += 360
                
                pidController = CircularPIDController(0.3,0,0,360)
                calculatedAngle = pidController.calculate(goal_angle, current_angle)
                if abs(calculatedAngle) > 1.8:
                    motor.move_angle(calculatedAngle)
                    current_angle += calculatedAngle
        except ValueError as error:
            print('Error in rotation mechanism: ' + str(error))

def main()->None:
    global pi_state, ads1115, bme280, mpu6050, guenther

    start_time = datetime.datetime.now()
    start_perf = round(perf_counter() * 1000)
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
    mpu6050_accelerations = []
    rotationangles = []
    
    Thread(target=rotation_mechanism, args=()).start()
    sleep(1)
    
    while True:
        timestamp = round(perf_counter() * 1000 - start_perf)
        timestamps.append(timestamp)
        status_led.off()
        
        if len(timestamps) > 1:
            time_difference = timestamp - timestamps[-2]
            
            if time_difference > max_iteration_time:
                print(f'Warning: Single iteration took more than {max_iteration_time}ms (took {time_difference}ms)')

        print(f'Luminance at sensors (lux): {luminance1} {luminance2} {luminance3}')

        try:
            ads1115_value = ads1115.read_adc(0, gain=1)
            solar_voltage = round(4.096 / 32767 * ads1115_value, 3)
            print(f'Solar panel voltage: {solar_voltage}V')
        except Exception as error:
            # try to contact sensor again
            print(str(error))
            ads1115 = initialize_ads1115()
        
        # X means they were not set yet
        pressure = 'X'
        temperature = 'X'
        
        try:
            bme280.update_sensor()
            pressure = round(float(bme280.pressure), 2)
            pressures.append(pressure)
            temperature = round(float(bme280.temperature), 2)
            temperatures.append(temperature)
            humidity = round(float(bme280.humidity) / 100, 2)
            humidities.append(humidity)
            altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
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
            bme280 = initialize_bme280()
    
        try:
            gyro_data = mpu6050.get_scaled_gyroscope()
            rotationrates.append(gyro_data)
            rotationrate_x, rotationrate_y, rotationrate_z = gyro_data
            acceleration = mpu6050.get_scaled_acceleration()
            mpu6050_accelerations.append(acceleration)
            acceleration_x, acceleration_y, acceleration_z = acceleration
            rotation = mpu6050.get_rotation(*acceleration)
            rotationangles.append(rotation)
            rotation_x, rotation_y = rotation
            
            print(f'Rate of rotation (°/s): {rotationrate_x}x {rotationrate_y}y {rotationrate_z}z')
            print(f'Static Acceleration (g): {acceleration_x}x {acceleration_y}y {acceleration_z}z')
            print(f'Angle of rotation (°): {rotation_x}x {rotation_y}y')
        except Exception as error:
            # try to contact sensor again
            mpu6050 = initialize_mpu6050()
        
        try:
            guenther.send(f'{CANSAT_ID};{timestamp};{pressure};{temperature}')
        except Exception as error:
            # try to contact transceiver again
            print(str(error))
            guenther = initialize_guenther()
        
        status_led.on()

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
                status_led.on()
                exit()
            else:
                pi_state = 'running'

                while blinking:
                    pass 

                status_led.on()

        sleep(1)

try:
    pi_state = 'running'
    print('Program is running')

    while blinking:
        pass

    status_led.on()

    # wait until power button is let go
    while power_button.is_pressed:
        pass

    main()

except KeyboardInterrupt:
    print('Running program stopped by keyboard interrupt')
    status_led.off()

finally:
    if motor is not None:
        motor.set_angle(0)
    
    GPIO.cleanup()
    pi_state = 'off'
    currentTime = datetime.datetime.now()
    currentTimeStr = currentTime.strftime("%H:%M:%S")
    guenther.send('(Info) CanSat program finished at: ' + currentTimeStr) 
    print('bye bye')
