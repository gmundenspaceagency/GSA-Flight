import os
import csv
import math
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
from mpu6050 import mpu6050 as Mpu6050
from Adafruit_ADS1x15 import ADS1115
from gsa_components.multiplexer import Multiplexer
from gsa_components.bh1750 import Bh1750
from gsa_components.motor import StepperMotor
from gsa_components.rak4200 import Rak4200
from gsa_components.gt_u7 import Gt_u7
from utils.persistent_bool import PersistentBool
from tests import CircularPIDController

"""
Allgemeine ToDos:
- Bodenstation und Transceiver
    - Empfangene Daten in einem bodenstation-log.csv speichern
    - Kleine UI mit empfangenen Daten, mögliche Fehler Codes darstellen
- Arbeit mit Daten nach Flug
    - Wenn schon Flight-Log erstellt, probieren die Daten in coolen Diagrammen darzustellen
- Script noch sicherer machen, ALLE EXCEPTIONS müssen aufgefangen werden
"""

CANSAT_ID = "69xd"
MODE = "groundtest" # either "groundtest", "modetest" or "flight"

# values for groundtest in seconds:
ground_duration = 5
ascending_duration = 5
descending_duration = 5

deactivate_beeping = False
pi_state = "initializing"
current_errors = []
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
        
        try:
            while pi_state == state:
                for timeout in seq:
                    status_led.toggle()
                    sleep(timeout)
        except Exception as error:
            print("Error in blinking thread: " + str(error))

        blinking = False

    # 3x fast blinking to show pi is initializing
    Thread(target=blink_status, args=([0.1]*7 + [0.5], "initializing")).start()
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
        current_errors.append("1")
    return light1

def set_multiplexer_channel(channel:int)->None:
    global multiplexer

    try:
        multiplexer.switch_to(channel)
    except Exception as error:
        # try to contact module again
        multiplexer = initialize_multiplexer()

def format_num(num:float)->str:
    return str(num).replace(".", ",")

def cleanup()->None:
    beeper.off()
    status_led.off()
    # TODO: motor disablen

def dist(a, b):
    return math.sqrt((a*a)+(b*b))

def get_rotation(x, y, z):
    xradians = math.atan2(y, dist(x, z))
    xrotation = round(math.degrees(xradians), 2)
    yradians = math.atan2(x, dist(y, z))
    yrotation = round(-math.degrees(yradians), 2)
    return xrotation, yrotation

def initialize_light2n3()->Optional[Bh1750]:
    set_multiplexer_channel(6)
   
    try:
        light2n3 = Bh1750()
        light2n3.luminance(Bh1750.ONCE_LOWRES)
    except Exception as error:
        light2n3 = None
        print("Problem with light sensor 2 or 3: " + str(error))
        current_errors.append("2")
    return light2n3

def initialize_ads1115()->Optional[ADS1115]:
    try:
        ads1115 = ADS1115()
        ads1115.read_adc(0, gain=1)
    except Exception as error:
        ads1115 = None
        print("Problem with A/D-Converter: " + str(error))
        current_errors.append("3")
    return ads1115

def initialize_bme280()->Optional[BME280]:
    try:
        bme280 = BME280()
        bme280.get_temperature()
    except Exception as error:
        bme280 = None
        print("Problem with BME280-Sensor: " + str(error))
        current_errors.append("4")
    return bme280
 
def initialize_mpu6050()->Optional[Mpu6050]:
    try:
        mpu6050 = Mpu6050(0x68)
        mpu6050.get_accel_data()
    except Exception as error:
        mpu6050 = None
        print("Problem with gyroscope: " + str(error))
        current_errors.append("5")
    return mpu6050

mpu6050_error = {"x": 10.136, "y": 1.657, "z": 1.225}

def initialize_motor()->Optional[StepperMotor]:
    try:
        motor = StepperMotor(24, 23)
        # sleep(0.5)
        # motor.set_angle(270)
        # sleep(0.5)
        # motor.set_angle(0)
    except Exception as error:
        motor = None
        print("Problem with motor: " + str(error))
        current_errors.append("6")
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
        current_errors.append("X")
    return guenther

def initialize_camera()->Optional[picamera.PiCamera]:
    try:
        camera = picamera.PiCamera()
    except Exception as error:
        camera = None
        print("Problem with camera: " + str(error))
        current_errors.append("7")
    return camera

def initialize_multiplexer()->Optional[Multiplexer]:
    try:
        multiplexer = Multiplexer()
    except Exception as error:
        multiplexer = None
        print("Problem with multiplexer: " + str(error))
        current_errors.append("7") 
    return multiplexer

def initialize_gt_u7()->Optional[Gt_u7]:
    try:
        gps = Gt_u7()
        gps.get_coordinates()
    except Exception as error:
        gps = None
        print("Problem with gps: " + str(error))
        current_errors.append("8")
    return gps
try:
    GPIO.setmode(GPIO.BCM)

    # not using Button class of gpiozero because of weird error when starting from crontab
    power_button = 10
    GPIO.setup(power_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    beeper = LED(20 if deactivate_beeping else 21)
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
    pi_state = "off"
    cleanup()
    exit()
try:
    pi_state = "ready"
    print(f"Pi is ready in {MODE} mode, hold start button to start program")

    # wait until the second thread with the blink_status function has stopped
    while blinking:
        pass

    # blinking to show the pi is ready
    Thread(target=blink_status, args=([0.5], "ready")).start()

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
    pi_state = "off"
    cleanup()
    exit()

luminance1 = luminance2 = luminance3 = solar_voltage = goal_angle = None

def rotation_mechanism() -> None:
    global light1, light2n3, multiplexer, luminance1, luminance2, luminance3, solar_voltage, goal_angle, motor
    last_total_luminance = 80000
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

            # TODO: test this code when one, two and three sensors dont work
            luminances = [luminance1, luminance2, luminance3]
            # TODO: check if the value 1000 works on a rainy day
            valid_luminances = [lum for lum in luminances if lum is not None and lum >= 1000]
            total_luminance = sum(valid_luminances)

            if len(valid_luminances) == 2:
                diff = last_total_luminance - total_luminance
                for i in range(len(luminances)):
                    if luminances[i] is None or luminances[i] < 1000:
                        luminances[i] = diff
                        break
            
            last_total_luminance = sum(luminances)
            
            # TODO: do we still need this if statement and why is the luminance list set again?
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
                print(calculatedAngle, goal_angle)
            
            try:
                ads1115_value = ads1115.read_adc(0, gain=1)
                solar_voltage = round(4.096 / 32767 * ads1115_value, 3)
            except Exception as error:
                # try to contact sensor again
                ads1115 = initialize_ads1115()

        except Exception as error:
            print("Error in rotation mechanism: " + str(error))

            if "9" not in current_errors:
                current_errors.append("9")
            
            # dont overload cpu
            sleep(0.1)

def main()->None:
    global pi_state, ads1115, bme280, mpu6050, guenther, camera, multiplexer, gps, luminance1, luminance2, luminance3, solar_voltage, current_errors, motor, goal_angle

    start_time = datetime.datetime.now()
    start_time_str = start_time.strftime("%Y-%m-%d_%H-%M-%S")
    start_perf = round(perf_counter() * 1000)
    timestamps = []
    bme_altitudes = []
    gps_altitudes = []
    vertical_speeds = []
    vertical_accelerations = []
    # times in milliseconds
    max_iteration_time = 1000
    goal_iteration_time = 500
    fake_start_altitude = 266.0

    strong_acceleration_start = None

    bme_ascending_check = PersistentBool()
    gps_ascending_check = PersistentBool()
    gyro_ascending_check = PersistentBool()
   
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
        sleep(1)
        bme280.update_sensor()
        pressure = round(float(bme280.pressure), 2)
        start_bme_altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
    except Exception as error:
        # try to contact sensor again
        bme280 = initialize_bme280()
        start_bme_altitude = fake_start_altitude
    
    try:
        start_gps_altitude = gps.get_altitude()
    except Exception as error:
        # try to contact module again
        guenther = initialize_guenther()
        start_gps_altitude = fake_start_altitude

    logfile_path = log_dir + "datalog.csv"

    with open(logfile_path, "a") as logfile:
        csv_writer = csv.writer(logfile, delimiter = ";")
        csv_writer.writerow(["Timestamp", "Pressure (hPa)", "Temperature (°C)", "Humidity (%)", "Altitude (m)","Speed (m/s)", "Relative Vertical Acceleration (m/s^2)", "Absolute Acceleration X (m/s^2)","Absolute Acceleration Y (m/s^2)", "Absolute Acceleration Z (m/s^2)", "Rate of Rotation X (°/s)","Rate of Rotation Y (°/s)", "Rate of Rotation Z (°/s)", "Rotation X (°)", "Rotation Y (°)", "Motor Rotation (°)","Luminance at 0° (lux)", "Luminance at 120° (lux)", "Luminance at 240° (lux)","Calculated Light Angle (°)", "Solar Panel Voltage (V)", "Gps Lat (°)", "Gps Lon (°)","Gps Altitude (m)", "Errors", "Status"])
    
    with open(log_dir + "info.txt", "a") as logfile:
        logfile.write(f"CanSat logdata - Team Gmunden Space Agency\nTimestamp: {start_time_str}\nMode: {MODE}\nStart altitude: {start_bme_altitude}m (BME280) {start_gps_altitude}m (GPS)")

    while pi_state == "ground_level":
        status_led.off()
        current_errors.clear()
        timestamp = round(perf_counter() * 1000 - start_perf)
        timestamps.append(timestamp)
        bme_altitude = gps_altitude = pressure = temperature = humidity = gps_lat = gps_lon = z_acceleration = None
        highest_z_acceleration = 0

        try:
            bme280.update_sensor()
            pressure = round(float(bme280.pressure), 2)
            bme_altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
            bme_altitudes.append(bme_altitude)
            temperature = round(float(bme280.temperature), 2)
            humidity = round(float(bme280.humidity) / 100, 2)

            if start_bme_altitude == fake_start_altitude:
                start_bme_altitude = bme_altitude
        except Exception as error:
            # try to contact sensor again
            bme280 = initialize_bme280()

        try:
            gps_lat, gps_lon = gps.get_coordinates()
            gps_altitude = gps.get_altitude()

            if start_gps_altitude == fake_start_altitude:
                start_gps_altitude = gps_altitude
        except Exception as error:
            gps = initialize_gt_u7()
        
        try:
            z_acceleration = mpu6050.get_accel_data()["z"]
            # using abs because cansat could be in rocket upside down
            highest_z_acceleration = max(highest_z_acceleration, abs(z_acceleration))
        except Exception as error:
            # try to contact sensor again
            mpu6050 = initialize_mpu6050()

        with open(logfile_path, "a") as logfile:
            csv_writer = csv.writer(logfile, delimiter=";")
            data = [
                timestamp,
                pressure,
                temperature,
                humidity,
                bme_altitude,
                "",
                "",
                "",
                "",
                z_acceleration,
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                gps_lat,
                gps_lon,
                gps_altitude,
                ",".join(current_errors),
                pi_state
            ]
            
            csv_writer.writerow([format_num(value) for value in data])

        bme_altitude_diff = 10
        gps_altitude_diff = 10
        max_z_acceleration = 150 # m/s^2 ~= 15g
        if (bme_altitude > start_bme_altitude + bme_altitude_diff if bme_altitude is not None else False):
            bme_ascending_check.set_true()
        if (gps_altitude > start_gps_altitude + gps_altitude_diff if gps_altitude is not None else False):
            gps_ascending_check.set_true()

        if strong_acceleration_start is None and highest_z_acceleration >= max_z_acceleration:
            strong_acceleration_start = timestamp
        if strong_acceleration_start is not None and highest_z_acceleration < max_z_acceleration:
            strong_acceleration_start = None
        if strong_acceleration_start is not None and timestamp - strong_acceleration_start > 3000:
            gyro_ascending_check.set_true()
                
        # use "or" for ascending checks, because we cant rely on all data being correct (better false start than no start)
        if (bme_ascending_check.state or gps_ascending_check.state or gyro_ascending_check.state) or (MODE == "groundtest" and len(timestamps) > ground_duration):
            pi_state = "ascending"
        
        try:
            guenther.send(f"(Info) Chilling on ground level at {bme_altitude}m (BME280) and {gps_altitude}m (GPS)")
        except Exception as error:
            # try to contact transceiver again
            guenther = initialize_guenther()
        
        status_led.on()
        bme_ascending_check.update()
        gps_ascending_check.update()
        gyro_ascending_check.update()
        sleep(1) # sleep longer than goal_iteration_time to save power

    start_recording()
    start_ascend_timestamp = round(perf_counter() * 1000 - start_perf)
    lowest_z_acceleration = avg_highest_luminance = above_avg_luminance_count = 0
    highest_luminance_values = []
    descending_speed_bool = PersistentBool()
    gps_negative_speed_bool = PersistentBool(persist_duration=3)
    avg_bme_vertical_speed_bool = PersistentBool(persist_duration=3)
    vertical_acceleration_bool = PersistentBool(persist_duration=3)

    while pi_state == "ascending":
        try:
            status_led.off()
            current_errors.clear()
            timestamp = round(perf_counter() * 1000 - start_perf)
            timestamps.append(timestamp)
            
            if camera is not None:
                camera = initialize_camera()
                start_recording()

            if len(timestamps) > 1:
                time_difference = timestamp - timestamps[-2]
                
                if time_difference > max_iteration_time:
                    print(f"Warning: Single iteration in descent took more than {max_iteration_time}ms (took {time_difference}ms)")
                    current_errors.append("F")
                        
            pressure = temperature = bme_altitude = humidity = vertical_speed = vertical_acceleration = gps_lat = gps_lon = None
            acceleration_x = acceleration_y = acceleration_z = rotation_x = rotation_y = rotationrate_x = rotationrate_y = rotationrate_z = None
            avg_bme_vertical_speed = gps_altitude = gps_speed = rotation_x = rotation_y = None

            try:
                bme280.update_sensor()
                pressure = round(float(bme280.pressure), 2)
                temperature = round(float(bme280.temperature), 2)
                humidity = round(float(bme280.humidity) / 100, 2)
                bme_altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
                bme_altitudes.append(bme_altitude)
                print(f"Pressure: {pressure}hPa, temperature: {temperature}°C, humidity: {round(humidity * 100, 2)}%")

                # speed can only be calculated after 2 height measures
                if len(timestamps) > 1:
                    avg_over = 3
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
                gyro_data = mpu6050.get_gyro_data()
                rotationrate_x = gyro_data["x"] - mpu6050_error["x"]
                rotationrate_y = gyro_data["y"] - mpu6050_error["y"]
                rotationrate_z = gyro_data["z"] - mpu6050_error["z"]
                acceleration = mpu6050.get_accel_data()
                acceleration_x = acceleration["x"]
                acceleration_y = acceleration["y"]
                acceleration_z = acceleration["z"]
                lowest_z_acceleration = min(lowest_z_acceleration, acceleration_z)
                rotation = get_rotation(acceleration_x, acceleration_y, acceleration_z)
                rotation_x, rotation_y = rotation
                
                print(f"Rate of rotation (°/s): {rotationrate_x}x {rotationrate_y}y {rotationrate_z}z")
                print(f"Static Acceleration (m/s^2): {acceleration_x}x {acceleration_y}y {acceleration_z}z")
                print(f"Angle of rotation (°): {rotation_x}x {rotation_y}y")
            except Exception as error:
                # try to contact sensor again
                mpu6050 = initialize_mpu6050()
            
            luminance1 = luminance2 = luminance3 = 0

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

            highest_luminance = max([luminance1, luminance2, luminance3])
            highest_luminance_values.append(highest_luminance)
            all_light_sensors_broken = highest_luminance == 0

            if highest_luminance_values:
                avg_highest_luminance = sum(highest_luminance_values) / len(highest_luminance_values)
                
            try:
                gps_lat, gps_lon = gps.get_coordinates()
                gps_altitude = gps.get_altitude()
                gps_altitudes.append(gps_altitude)

                if gps_altitude is not None:
                    # search last not None altitude
                    for i in range(1, len(gps_altitudes)):
                        if gps_altitudes[-i] is not None:
                            gps_speed = (gps_altitude - gps_altitudes[-i]) / (timestamp - timestamps[-i])
                            break

            except Exception as error:
                gps = initialize_gt_u7()
            

            if (gps_speed < -2 if gps_speed is not None else False):
                gps_negative_speed_bool.set_true()
            if (avg_bme_vertical_speed < -2 if avg_bme_vertical_speed is not None else False):
                avg_bme_vertical_speed_bool.set_true()
            # cansat should have < 0m/s^2 static acceleration in free fall
            if lowest_z_acceleration < -10:
                vertical_acceleration_bool.set_true()
            if highest_luminance > avg_highest_luminance + 500 or all_light_sensors_broken:
                above_avg_luminance_count += 1

            bme_works = bme_altitude is not None
            gps_works = gps_altitude is not None
            gyro_works = acceleration_z is not None

            working_sensors_count = bme_works + gps_works + gyro_works
            descending_indicators_count = gps_negative_speed_bool.state + avg_bme_vertical_speed_bool.state + vertical_acceleration_bool.state
            
            # if at least half of the working sensors indicate a descent, descending mode is activated
            if descending_indicators_count >= working_sensors_count - 1:
                descending_speed_bool.set_true()

            if (
                (
                    (
                        timestamp - start_ascend_timestamp > 3000 # ascend must be longer than 3s
                        and descending_speed_bool.state # half of working sensors indicate a descent
                        and above_avg_luminance_count >= 2 # the light sensors are above average luminance for at least 2 iteration or dont work
                    )
                    or (timestamp - start_ascend_timestamp > 7000 and MODE != "modetest") # max ascending time is 7s
                ) 
                or (MODE == "groundtest" and len(timestamps) > ground_duration + ascending_duration)
            ):
                pi_state = "descending"
              
            try:
                error_string = "".join(current_errors)
                guenther.send(f"{CANSAT_ID};{timestamp};{pressure};{temperature};{error_string};C") # C for ascending
            except Exception as error:
                # try to contact transceiver again
                guenther = initialize_guenther()

            with open(logfile_path, "a") as logfile:
                csv_writer = csv.writer(logfile, delimiter=";")
                data = [
                    timestamp,
                    pressure,
                    temperature,
                    humidity,
                    bme_altitude,
                    vertical_speed,
                    vertical_acceleration,
                    acceleration_x,
                    acceleration_y,
                    acceleration_z,
                    rotationrate_x,
                    rotationrate_y,
                    rotationrate_z,
                    rotation_x,
                    rotation_y,
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    gps_lat,
                    gps_lon,
                    gps_altitude,
                    ",".join(current_errors),
                    pi_state
                ]

                csv_writer.writerow([format_num(value) for value in data])

            status_led.on()
        except Exception as error:
            print("Error in ascending loop: " + str(error))
            current_errors.append("A")

        current_iteration_duration = round(perf_counter() * 1000 - start_perf) - timestamp
        sleep(max(0, (goal_iteration_time - current_iteration_duration) / 1000))

    rotation_thread = Thread(target=rotation_mechanism, args=())
    rotation_thread.start()
    print("CanSat has started falling")
    start_descending_timestamp = round(perf_counter() * 1000 - start_perf)

    while pi_state == "descending":
        try:
            status_led.off()
            current_errors.clear()
            timestamp = round(perf_counter() * 1000 - start_perf)
            timestamps.append(timestamp)

            if camera is None:
                # try to contact sensor again
                camera = initialize_camera()
                start_recording() 
            
            if len(timestamps) > 1:
                time_difference = timestamp - timestamps[-2]
                
                if time_difference > max_iteration_time:
                    print(f"Warning: Single iteration took more than {max_iteration_time}ms (took {time_difference}ms)")
                    current_errors.append("F")

            pressure = temperature = bme_altitude = humidity = vertical_speed = vertical_acceleration = rotation_x = rotation_y = None
            acceleration_x = acceleration_y = acceleration_z = rotation_x = rotation_y = rotationrate_x = rotationrate_y = rotationrate_z = gps_lat = gps_lon = None

            print(f"Luminance at sensors (lux): {luminance1} {luminance2} {luminance3}")
            print(f"Solar panel voltage: {solar_voltage}V")
            
            try:
                bme280.update_sensor()
                pressure = round(float(bme280.pressure), 2)
                temperature = round(float(bme280.temperature), 2)
                humidity = round(float(bme280.humidity) / 100, 2)
                bme_altitude = round(44330.0 * (1.0 - pow(pressure / 1013.25, (1.0 / 5.255))), 2)
                bme_altitudes.append(bme_altitude)

                print(f"Pressure: {pressure}hPa, temperature: {temperature}°C, humidity: {round(humidity * 100, 2)}%")

                # speed can only be calculated after 2 height measures
                if len(timestamps) > 1:
                    avg_over = 3

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
                gyro_data = mpu6050.get_gyro_data()
                rotationrate_x = gyro_data["x"]
                rotationrate_y = gyro_data["y"]
                rotationrate_z = gyro_data["z"]
                acceleration = mpu6050.get_accel_data()
                acceleration_x = acceleration["x"]
                acceleration_y = acceleration["y"]
                acceleration_z = acceleration["z"]
                lowest_z_acceleration = min(lowest_z_acceleration, acceleration_z)
                rotation = get_rotation(acceleration_x, acceleration_y, acceleration_z)
                rotation_x, rotation_y = rotation
                
                print(f"Rate of rotation (°/s): {rotationrate_x}x {rotationrate_y}y {rotationrate_z}z")
                print(f"Static Acceleration (m/s^2): {acceleration_x}x {acceleration_y}y {acceleration_z}z")
                print(f"Angle of rotation (°): {rotation_x}x {rotation_y}y")
            except Exception as error:
                # try to contact sensor again
                mpu6050 = initialize_mpu6050()

            try:
                gps_lat, gps_lon = gps.get_coordinates()
                gps_altitude = gps.get_altitude()              
            except Exception as error:
                # try to contact module again
                gps = initialize_gt_u7()

            if (
                (MODE == "flight" and timestamp - start_descending_timestamp > 60000) # always descend for 1 minute
                or (MODE == "modetest" and bme_altitude is not None and bme_altitude < start_bme_altitude + 10) # or reach ground again in modetest
                or (MODE == "groundtest" and len(timestamps) > ground_duration + ascending_duration + descending_duration)
            ):
                pi_state = "landed"
                    
            try:
                error_string = "".join(current_errors)
                guenther.send(f"{CANSAT_ID};{timestamp};{pressure};{temperature};{error_string};D") # D for descending
            except Exception as error:
                # try to contact transceiver again
                guenther = initialize_guenther()
            
            with open(logfile_path, "a") as logfile:
                csv_writer = csv.writer(logfile, delimiter=";")
                data = [
                    timestamp,
                    pressure,
                    temperature,
                    humidity,
                    bme_altitude,
                    vertical_speed,
                    vertical_acceleration,
                    acceleration_x,
                    acceleration_y,
                    acceleration_z,
                    rotationrate_x,
                    rotationrate_y,
                    rotationrate_z,
                    rotation_x,
                    rotation_y,
                    motor.current_angle,
                    luminance1,
                    luminance2,
                    luminance3,
                    goal_angle,
                    solar_voltage,
                    gps_lat,
                    gps_lon,
                    gps_altitude,
                    ",".join(current_errors),
                    pi_state
                ]

                csv_writer.writerow([format_num(value) for value in data])
                               
            status_led.on()
        except Exception as error:
            print("Error in descending loop: " + str(error))
            current_errors.append("A")

        current_iteration_duration = round(perf_counter() * 1000 - start_perf) - timestamp
        sleep(max(0, (goal_iteration_time - current_iteration_duration) / 1000))

    print("CanSat has reached ground level")

    if camera is not None:
        try:
            camera.stop_recording()
            camera.stop_preview()
        except Exception as error:
            print("Stopping of camera recording didn't work: " + str(error))
    
    while pi_state == "landed":
        status_led.off()
        timestamp = round(perf_counter() * 1000 - start_perf)
        timestamps.append(timestamp)
        beeper.toggle()
        current_errors.clear()
        gps_lat = gps_lon = gps_altitude = None

        try:
            gps_lat, gps_lon = gps.get_coordinates()
            gps_altitude = gps.get_altitude()
        except Exception as error:
            # try to contact module again
            gps = initialize_gt_u7()

        with open(logfile_path, "a") as logfile:
            csv_writer = csv.writer(logfile, delimiter=";")
            data = [
                timestamp,
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                gps_lat,
                gps_lon,
                gps_altitude,
                "".join(current_errors),
                pi_state
            ]

            csv_writer.writerow([format_num(value) for value in data])

        try:
            guenther.send(f"(Info) Landed at: {gps_lat}N {gps_lon}E {gps_altitude}m")
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
    print(f"Program is running in {MODE} mode")

    while blinking:
        pass

    status_led.on()

    # wait until power button is let go
    while GPIO.input(power_button) == GPIO.LOW:
        pass

    beeper.on()
    sleep(0.1)
    beeper.off()
    sleep(0.1)
    beeper.on()
    sleep(0.1)
    beeper.off()

    main()

except KeyboardInterrupt:
    print("Running program stopped by keyboard interrupt")
    status_led.off()

finally:
    if motor is not None:
        motor.set_angle(0)
        
    cleanup()
    pi_state = "off"
    currentTime = datetime.datetime.now()
    currentTimeStr = currentTime.strftime("%H:%M:%S")
    
    if guenther is not None:
        guenther.send("(Info) CanSat program finished at: " + currentTimeStr)
        
    print("bye bye")
