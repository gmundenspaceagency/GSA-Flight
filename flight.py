import os
import time
import smbus
import datetime
import picamera
import RPi.GPIO as GPIO

from typing import Optional
from threading import Thread
from gpiozero import LED
from time import sleep, perf_counter

# pimoroni-bme280 1.0.0
from bme280 import BME280
from Adafruit_ADS1x15 import ADS1115
from gsa_components.mpu6050 import Mpu6050
from gsa_components.multiplexer import Multiplexer
from gsa_components.bh1750 import Bh1750
from gsa_components.motor import StepperMotor
from gsa_components.rak4200 import Rak4200
from gsa_components.gt_u7 import Gt_u7
from tests import CircularPIDController

"""
Allgemeine ToDos:
- Programm stoppt einfach wenn guenther nicht gefunden wird (timeout?)
- GPS Daten lesen, an die Bodenstation senden ob wir FIX haben oder nicht
    - GPS Höhendaten mit dem bme280280 Daten abgleichen
- Temperaturdaten von GYRO und bme280280 abgleichen
- Error handling
    - Wenn ein Error zum ersten Mal auftritt: speichern wann er aufgetreten ist, den Fehlertext (str(error)), und wenn er wieder weggeht speicher wann er weggegangen ist
    - Trotzdem noch alle Fehler printen
    - In einer JSON Datei allen unseren Fehlern error codes zuordnen (eg. 01 -> Fehler mit Lichtsensor1, 08 -> Fehler mit bme280280)
        - In einem Array aktuell aktive Fehler speichern und dieses Array wenn möglich zur Bodenstation schicken
- Bodenstation und Transceiver
    - Empfangene Daten in einem bodenstation-log.csv speichern
    - Kleine UI mit empfangenen Daten, mögliche Fehler Codes darstellen
- Arbeit mit Daten nach Flug
    - Wenn schon Flight-Log erstellt, probieren die Daten in coolen Diagrammen darzustellen
- Script noch sicherer machen, ALLE EXCEPTIONS müssen aufgefangen werden
"""

CANSAT_ID = "69xd"
MODE = "groundtest" # either "groundtest" or "flight"
pi_state = "initializing"
print("Pi is initializing...")

# is True when a second thread with the blink_status function is running
blinking = False

# initialize and test status led
try:
    status_led = LED(25)

    # controls blinking of status led to indicate different states of pi
    def blink_status(seq: list[float], state: str) -> None:
        global blinking
        blinking = True

        while pi_state == state:
            for timeout in seq:
                status_led.toggle()
                sleep(timeout)

        blinking = False

    # blinking to show pi is initializing
    Thread(target=blink_status, args=([0.5], "initializing")).start()
except Exception as error:
    print("Warning: Error while initializing onboard LED: " + str(error))

# initialize sensors
def initialize_light1()->Optional[Bh1750]:
    try:
        light1 = Bh1750(1, 0x5c)
        light1.luminance(Bh1750.ONCE_LOWRES)
    except Exception as error:
        light1 = None
        print("Problem with light sensor 1: " + str(error))
    
    return light1

def set_multiplexer_channel(channel:int)->None:
    try:
        multiplexer.switch_to(channel)
    except Exception as error:
        # try to contact module again
        multiplexer = initialize_multiplexer()

def initialize_light2n3()->Optional[Bh1750]:
    set_multiplexer_channel(6)
   
    try:
        light2n3 = Bh1750()
        light2n3.luminance(Bh1750.ONCE_LOWRES)
    except Exception as error:
        light2n3 = None
        print("Problem with light sensor 2 or 3: " + str(error))
    
    return light2n3

def initialize_ads1115()->Optional[ADS1115]:
    try:
        ads1115 = ADS1115()
        ads1115.read_adc(0, gain=1)
    except Exception as error:
        ads1115 = None
        print("Problem with A/D-Converter: " + str(error))
    
    return ads1115

def initialize_bme280()->Optional[BME280]:
    try:
        bme280 = BME280()
        bme280.get_temperature()
    except Exception as error:
        bme280 = None
        print("Problem with BME280-Sensor: " + str(error))
    
    return bme280
 
def initialize_mpu6050()->Optional[Mpu6050]:
    try:
        mpu6050 = Mpu6050()
        mpu6050.get_scaled_acceleration()
    except Exception as error:
        mpu6050 = None
        print("Problem with gyroscope: " + str(error))
    
    return mpu6050

def initialize_motor()->Optional[StepperMotor]:
    try:
        motor = StepperMotor(26, 19, 200, 0.002)
        # sleep(0.5)
        # motor.set_angle(270)
        # sleep(0.5)
        # motor.set_angle(0)
    except Exception as error:
        motor = None
        print("Problem with motor: " + str(error))
    
    return motor

"""
Günther is the TRANSCEIVER!!!
"""

def initialize_guenther()->Optional[Rak4200]:
    try:
        guenther = Rak4200()
        guenther.start("send")
    except Exception as error:
        guenther = None
        print("Problem with guenther: " + str(error))
    
    return guenther

def initialize_camera()->Optional[picamera.PiCamera]:
    try:
        camera = picamera.PiCamera()
    except Exception as error:
        camera = None
        print("Problem with camera: " + str(error))
    
    return camera

def initialize_multiplexer()->Optional[Multiplexer]:
    try:
        multiplexer = Multiplexer()
    except Exception as error:
        multiplexer = None
        print("Problem with multiplexer: " + str(error))
    return multiplexer

def initialize_gt_u7()->Optional[Gt_u7]:
    try:
        gps = Gt_u7()
    except Exception as error:
        gps = None
        print("Problem with gps: " + str(error))
    return gps
try:
    GPIO.setmode(GPIO.BCM)

    # not using Button class of gpiozero because of weird error when starting from crontab
    power_button = 10
    GPIO.setup(power_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    beeper = LED(21)
    beeper.on()
    sleep(0.5)
    beeper.off()
    
    # every sensor is in a try-block so the program will still work when one of the sensors fails on launch
    multiplexer = initialize_multiplexer()
    light1 = initialize_light1()
    light2n3 = initialize_light2n3()
    ads1115 = initialize_ads1115()
    bme280 = initialize_bme280()
    mpu6050 = initialize_mpu6050()
    motor = initialize_motor()
    guenther = initialize_guenther()
    camera = initialize_camera()
    gps = initialize_gt_u7()
except KeyboardInterrupt:
    print("Initializing aborted by keyboard interrupt")
    GPIO.cleanup()
    exit()

try:
    pi_state = "ready"
    print("Pi is ready, hold start button to start program")

    # wait until the second thread with the blink_status function has stopped
    while blinking:
        pass

    # blinking to show the pi is ready
    Thread(target=blink_status, args=([0.1], "ready")).start()

    # wait for 1s button press
    while True:
        if GPIO.input(power_button) == GPIO.LOW:
            pi_state = "starting"

            while blinking:
                pass

            # fast flashing telling you should keep holding the button
            Thread(target=blink_status, args=([0.05], "starting")).start()
            sleep(1)

            if GPIO.input(power_button) == GPIO.LOW:
                break
            else:
                pi_state = "ready"

                while blinking:
                    pass

                Thread(target=blink_status, args=([0.5], "ready")).start()
except KeyboardInterrupt:
    print("Program stopped in ready state by keyboard interrupt")
    GPIO.cleanup()
    exit()

luminance1 = luminance2 = luminance3 = None

def rotation_mechanism() -> None:
    global light1, light2n3, multiplexer, luminance1, luminance2, luminance3
    
    while pi_state == "descending":            
        try:  
            try:
                luminance1 = round(light1.luminance(Bh1750.ONCE_LOWRES), 4)
            except Exception as error:
                # try to contact sensor again
                light1 = initialize_light1()
            try:
                set_multiplexer_channel(7)
                luminance2 = round(light2n3.luminance(Bh1750.ONCE_LOWRES), 4)
                set_multiplexer_channel(6)
                luminance3 = round(light2n3.luminance(Bh1750.ONCE_LOWRES), 4)
            except Exception as error:
                # try to contact sensor again
                light2n3 = initialize_light2n3()
            
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
                calculatedAngle = pidController.calculate(goal_angle, motor.current_angle)
                motor.move_angle(calculatedAngle)
        except ValueError as error:
            print("Error in rotation mechanism: " + str(error))

def main()->None:
    global pi_state, ads1115, bme280, mpu6050, guenther, camera, multiplexer, gps


    start_time = datetime.datetime.now()
    start_time_str = start_time.strftime("%Y-%m-%d_%H-%M-%S")
    start_perf = round(perf_counter() * 1000)
    # TODO: realistische iteration time
    timestamps = []
    bme_altitudes = []
    vertical_speeds = []
    vertical_accelerations = []
    max_iteration_time = 1000
    fake_start_altitude = 266.0
    
    log_dir = f"/home/gsa202324/GSA-Flight/log/flightlog_{start_time_str}/"
    os.mkdir(log_dir)
    video_output = log_dir + "flight-video.h264"
    resolution = (1920, 1080)
    
    def start_recording():
        if camera is not None:
            try:
                camera.resolution = resolution
                camera.start_preview()
                camera.start_recording(video_output)
            except Exception as error:
                print("Error in camera recording: " + str(error))
    try:
        bme280.update_sensor()
        pressure = round(float(bme280.pressure), 2)
        start_bme_altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
    except Exception as error:
        try:
            #TODO: error code statt ganzes teil senden + printen
            guenther.send(f"(Error) BME280 sensor not working: {error}")
        except Exception as error:
            guenther = initialize_guenther()

        start_bme_altitude = fake_start_altitude
    
    try:
        nmea_sentence = gps.serialPort.readline().decode().strip()
        start_gps_altitude = gps.extract_altitude(nmea_sentence)
    except Exception as error:
        try:
            #TODO: error code statt ganzes teil senden + printen
            guenther.send(f"(Error) GPS sensor not working: {error}")
        except Exception as error:
            guenther = initialize_guenther()
        
        start_gps_altitude = fake_start_altitude

    logfile_path = log_dir + "datalog.csv"

    with open(logfile_path, "a") as logfile:
        logfile.write(f"Teamname: Gmunden Space Agency, CanSat Flight Logfile at: {start_time}\n")
        logfile.write("\n")
        logfile.write("Timestamp,Pressure (hPa),Temperature (°C),Humidity (%),Altitudes (m),Speed (m/s),Relative Vertical Acceleration (m/s^2),Absolute Acceleration X (g),Absolute Acceleration Y (g),Absolute Acceleration Z (g),Rate of Rotation X (°/s),Rate of Rotation Y (°/s),Rate of Rotation Z (°/s),Motor Rotation (°),Luminance at 0° (lux),Luminance at 120° (lux),Luminance at 240° (lux),Calculated Light Angle (°),Solar Panel Voltage (V),GPS lat (°),GPS lon (°),Errors,Status\n")

    while pi_state == "ground_level":
        status_led.off()
        timestamp = round(perf_counter() * 1000 - start_perf)
        timestamps.append(timestamp)
        bme280.update_sensor()
        pressure = round(float(bme280.pressure), 2)
        bme_altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
        bme_altitudes.append(bme_altitude)
        temperature = round(float(bme280.temperature), 2)
        humidity = round(float(bme280.humidity) / 100, 2)
        try:
            nmea_sentence = gps.serialPort.readline().decode().strip()
            gps_lat_lon = gps.extract_lat_lon(nmea_sentence, "decimal")
            gps_altitude = gps.extract_altitude(nmea_sentence)
        except Exception as error:
            gps = initialize_gt_u7()

        if start_gps_altitude == fake_start_altitude:
            start_gps_altitude = gps_altitude
        
        if start_bme_altitude == fake_start_altitude:
            start_bme_altitude = bme_altitude

        with open(logfile_path, "a") as logfile:
            #TODO: gps daten loggen
            logfile.write(f"{timestamp},{pressure},{temperature},{humidity},{bme_altitude},,,,,,,,,,,,,,,{gps_lat_lon},None,{pi_state}\n")

        #TODO: wenn gps geht dann Gps beforzugen
        if (bme_altitude > start_bme_altitude + 10 and gps_altitude > start_gps_altitude + 10) or (MODE == "groundtest" and len(timestamps) > 5):
            pi_state = "ascending"
        
        try:
            guenther.send(f"(Info) Chilling on ground level")
        except Exception as error:
            # try to contact transceiver again
            guenther = initialize_guenther()
        
        status_led.on()
        sleep(1)

    start_recording()


    while pi_state == "ascending":
        status_led.off()
        errors = []
        timestamp = round(perf_counter() * 1000 - start_perf)
        timestamps.append(timestamp)
        
        if camera is not None:
            camera = initialize_camera()
            start_recording()

        if len(timestamps) > 1:
            time_difference = timestamp - timestamps[-2]
            
            if time_difference > max_iteration_time:
                print(f"Warning: Single iteration in descent took more than {max_iteration_time}ms (took {time_difference}ms)")
                errors.append(f"Warning: Single iteration in descent took more than {max_iteration_time}ms (took {time_difference}ms)")
        
        # X means they were not set yet
        pressure = temperature = bme_altitude = humidity = vertical_speed = vertical_acceleration = "X"
        acceleration_x = acceleration_y = acceleration_z = rotation_x = rotation_y = rotationrate_x = rotationrate_y = rotationrate_z = "X"
        
        try:
            bme280.update_sensor()
            pressure = round(float(bme280.pressure), 2)
            temperature = round(float(bme280.temperature), 2)
            humidity = round(float(bme280.humidity) / 100, 2)
            bme_altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
            bme_altitudes.append(bme_altitude)
            print(f"Pressure: {pressure}hPa, temperature: {temperature}°C, humidity: {humidity * 100}%")

            # speed can only be calculated after 2 height measures
            if len(timestamps) > 1:
                avg_over = 5
                avg_bme_vertical_speed = round(sum(vertical_speeds[max(-len(timestamps) + 1, -avg_over):]) / min(len(timestamps) - 1, avg_over), 2)
                vertical_speed = round((bme_altitude - bme_altitudes[-2]) / time_difference * 1000, 2)
                vertical_speeds.append(vertical_speed)
                print(f"Altitude: {bme_altitude}m, Speed: {vertical_speed}m/s, Average speed: {avg_bme_vertical_speed}m/s")      
            # acceleration can only be calculated after 3 height measures
            if len(timestamps) > 2:
                avg_vertical_acceleration = round(sum(vertical_accelerations[max(-len(timestamps) + 1, -avg_over):]) / min(len(timestamps) - 1, avg_over), 2)
                vertical_acceleration = round((vertical_speed - vertical_speeds[-2]) / time_difference, 3)
                vertical_accelerations.append(vertical_acceleration)


                print(f"Acceleration: {vertical_acceleration}m/s^2, Average acceleration: {avg_vertical_acceleration}m/s^2")
        except Exception as error:
            # try to contact sensor again
            bme280 = initialize_bme280()
    
        try:
            gyro_data = mpu6050.get_scaled_gyroscope()
            rotationrate_x, rotationrate_y, rotationrate_z = gyro_data
            acceleration = mpu6050.get_scaled_acceleration()
            acceleration_x, acceleration_y, acceleration_z = acceleration
            rotation = mpu6050.get_rotation(*acceleration)
            rotation_x, rotation_y = rotation
            
            print(f"Rate of rotation (°/s): {rotationrate_x}x {rotationrate_y}y {rotationrate_z}z")
            print(f"Static Acceleration (g): {acceleration_x}x {acceleration_y}y {acceleration_z}z")
            print(f"Angle of rotation (°): {rotation_x}x {rotation_y}y")
        except Exception as error:
            # try to contact sensor again
            mpu6050 = initialize_mpu6050()
        
        try:
            luminance1 = round(light1.luminance(Bh1750.ONCE_LOWRES), 4)
        except Exception as error:
            luminance1 = 0
            light1 = initialize_light1()

        try:
            set_multiplexer_channel(7)
            luminance2 = round(light2n3.luminance(Bh1750.ONCE_LOWRES), 4)
            set_multiplexer_channel(6)
            luminance3 = round(light2n3.luminance(Bh1750.ONCE_LOWRES), 4)
        except Exception as error:
            # try to contact sensor again
            luminance2 = 0
            luminance3 = 0
            light2n3 = initialize_light2n3()
        
        if luminance1 is not None and luminance2 is not None and luminance3 is not None:
            highest_luminance = max([luminance1, luminance3, luminance2])
        else:
            highest_luminance = 0
        
        try:
            nmea_sentence = gps.serialPort.readline().decode().strip()
            gps_lat_lon = gps.extract_lat_lon(nmea_sentence, "decimal")
            gps_altitude = gps.extract_altitude(nmea_sentence)
            gps_speed = gps.extract_velocity(nmea_sentence)
        except Exception as error:
            gps = initialize_gt_u7()

        #TODO: add max time and also gps height
        if (gps_speed and avg_bme_vertical_speed < -2 and vertical_acceleration < 0 and highest_luminance > 20) or (MODE == "groundtest" and len(timestamps) > 10):
            #TODO check for lighsensors
            pi_state = "descending"  
        try:
            guenther.send(f"{CANSAT_ID};{timestamp};{pressure};{temperature}")
        except Exception as error:
            # try to contact transceiver again
            guenther = initialize_guenther()

        with open(logfile_path, "a") as logfile:
            logfile.write(f"{timestamp},{pressure},{temperature},{humidity},{bme_altitude},{vertical_speed},{vertical_acceleration},{acceleration_x},{acceleration_y},{acceleration_z},{rotationrate_x},{rotationrate_y},{rotationrate_z},,,,,,,{gps_lat_lon},{';'.join(errors)},{pi_state}\n")
        
        status_led.on()
        sleep(1)

    rotation_thread = Thread(target=rotation_mechanism, args=())
    # rotation_thread.start()
    print("CanSat has started falling")
    start_descending_timestamp = round(perf_counter() * 1000 - start_perf)

    while pi_state == "descending":
        timestamp = round(perf_counter() * 1000 - start_perf)
        timestamps.append(timestamp)
        status_led.off()

        if camera is None:
            # try to contact sensor again
            camera = initialize_camera()
            start_recording() 
        
        if len(timestamps) > 1:
            time_difference = timestamp - timestamps[-2]
            
            if time_difference > max_iteration_time:
                print(f"Warning: Single iteration took more than {max_iteration_time}ms (took {time_difference}ms)")

        # X means they were not set yet
        pressure = temperature = bme_altitude = humidity = vertical_speed = vertical_acceleration = luminance1 = luminance2 = luminance3 = "X"
        acceleration_x = acceleration_y = acceleration_z = rotation_x = rotation_y = rotationrate_x = rotationrate_y = rotationrate_z = "X"

        print(f"Luminance at sensors (lux): {luminance1} {luminance2} {luminance3}")

        try:
            ads1115_value = ads1115.read_adc(0, gain=1)
            solar_voltage = round(4.096 / 32767 * ads1115_value, 3)
            print(f"Solar panel voltage: {solar_voltage}V")
        except Exception as error:
            # try to contact sensor again
            ads1115 = initialize_ads1115()
        
        # X means they were not set yet
        pressure = "X"
        temperature = "X"
        
        try:
            bme280.update_sensor()
            pressure = round(float(bme280.pressure), 2)
            temperature = round(float(bme280.temperature), 2)
            humidity = round(float(bme280.humidity) / 100, 2)
            bme_altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
            bme_altitudes.append(bme_altitude)

            print(f"Pressure: {pressure}hPa, temperature: {temperature}°C, humidity: {humidity * 100}%")

            # speed can only be calculated after 2 height measures
            if len(timestamps) > 1:
                avg_over = 5

                avg_bme_vertical_speed = round(sum(vertical_speeds[max(-len(timestamps) + 1, -avg_over):]) / min(len(timestamps) - 1, avg_over), 2)
                vertical_speed = round((bme_altitude - bme_altitudes[-2]) / time_difference * 1000, 2)
                vertical_speeds.append(vertical_speed)

                print(f"Altitude: {bme_altitude}m, Speed: {vertical_speed}m/s, Average speed: {avg_bme_vertical_speed}m/s")

            # acceleration can only be calculated after 3 height measures
            if len(timestamps) > 2:
                avg_vertical_acceleration = round(sum(vertical_accelerations[max(-len(timestamps) + 1, -avg_over):]) / min(len(timestamps) - 1, avg_over), 2)

                vertical_acceleration = round((vertical_speed - vertical_speeds[-2]) / time_difference, 3)
                vertical_accelerations.append(vertical_acceleration)

                print(f"Acceleration: {vertical_acceleration}m/s^2, Average acceleration: {avg_vertical_acceleration}m/s^2")
        except Exception as error:
            # try to contact sensor again
            bme280 = initialize_bme280()
    
        try:
            gyro_data = mpu6050.get_scaled_gyroscope()
            rotationrate_x, rotationrate_y, rotationrate_z = gyro_data
            acceleration = mpu6050.get_scaled_acceleration()
            acceleration_x, acceleration_y, acceleration_z = acceleration
            rotation = mpu6050.get_rotation(*acceleration)
            rotation_x, rotation_y = rotation
            
            print(f"Rate of rotation (°/s): {rotationrate_x}x {rotationrate_y}y {rotationrate_z}z")
            print(f"Static Acceleration (g): {acceleration_x}x {acceleration_y}y {acceleration_z}z")
            print(f"Angle of rotation (°): {rotation_x}x {rotation_y}y")
        except Exception as error:
            # try to contact sensor again
            mpu6050 = initialize_mpu6050()

        try:
            nmea_sentence = gps.serialPort.readline().decode().strip()
            gps_altitude = gps.extract_altitude(nmea_sentence)
            gps_lat_lon = gps.extract_lat_lon(nmea_sentence, "decimal")
        except Exception as error:
            #TODO report error to groundstation or in logs?
            gps = initialize_gt_u7()
        #gps mehr gewichten
        if (MODE != "groundtest" and bme_altitude < start_bme_altitude + 5 and gps_altitude < start_gps_altitude + 5) or timestamp - start_descending_timestamp > 30000 or (MODE == "groundtest" and len(timestamps) > 15):
                    pi_state = "landed"
                
        try:
            guenther.send(f"{CANSAT_ID};{timestamp};{pressure};{temperature}")
        except Exception as error:
            # try to contact transceiver again
            guenther = initialize_guenther()
        
        with open(logfile_path, "a") as logfile:
            logfile.write(f"{timestamp},{pressure},{temperature},{humidity},{bme_altitude},{vertical_speed},{vertical_acceleration},{acceleration_x},{acceleration_y},{acceleration_z},{rotationrate_x},{rotationrate_y},{rotationrate_z},,{luminance1},{luminance2},{luminance3},,,{gps_lat_lon},{';'.join(errors)},{pi_state}\n")
        
        status_led.on()
        sleep(1)

    print("CanSat has reached ground level")

    if camera is not None:
        try:
            camera.stop_recording()
            camera.stop_preview()
        except (KeyError, picamera.exc.PiCameraNotRecording):
            pass
    
    while pi_state == "landed":
        status_led.off()
        timestamp = round(perf_counter() * 1000 - start_perf)
        timestamps.append(timestamp)
        beeper.toggle()

        #TODO: add a landed mode with coordinates
        with open(logfile_path, "a") as logfile:
            logfile.write(f"{timestamp},,,,,,,,,,,,,,,,,,,None,{pi_state}\n")

        try:
            guenther.send(f"(Info) Back on the ground again")
        except Exception as error:
            # try to contact transceiver again
            guenther = initialize_guenther()
        
        status_led.on()

        # check for turn off
        if GPIO.input(power_button) == GPIO.LOW:
            pi_state = "shutting down"
            
            while blinking:
                pass
            
            # fast flashing telling pi is about to shut down
            Thread(target=blink_status, args=([0.05], "shutting down")).start()
            sleep(1)

            if GPIO.input(power_button) == GPIO.LOW:
                print("Program switched off with power button")
                status_led.on()
                exit()
            else:
                pi_state = "landed"

                while blinking:
                    pass 

                status_led.on()
        sleep(1)

try:
    pi_state = "ground_level"
    print("Program is running")

    while blinking:
        pass

    status_led.on()

    # wait until power button is let go
    while GPIO.input(power_button) == GPIO.LOW:
        pass

    main()

except KeyboardInterrupt:
    print("Running program stopped by keyboard interrupt")
    status_led.off()

finally:
    if motor is not None:
        motor.set_angle(0)
        
    GPIO.cleanup()
    pi_state = "off"
    currentTime = datetime.datetime.now()
    currentTimeStr = currentTime.strftime("%H:%M:%S")
    
    if guenther is not None:
        guenther.send("(Info) CanSat program finished at: " + currentTimeStr)
        
    print("bye bye")
