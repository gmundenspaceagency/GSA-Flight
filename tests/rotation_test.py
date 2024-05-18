import sys
sys.path.append('..')
sys.path.append('.')
from old_motor import Motor
from gsa_components.bh1750 import Bh1750
from gsa_components.multiplexer import Multiplexer
from time import sleep
from typing import Optional
from pid import CircularPIDController

motor = Motor()
last_total_luminance = 80000

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
    global multiplexer

    try:
        multiplexer.switch_to(channel)
    except Exception as error:
        # try to contact module again
        multiplexer = initialize_multiplexer()

def initialize_multiplexer()->Optional[Multiplexer]:
    try:
        multiplexer = Multiplexer()
    except Exception as error:
        multiplexer = None
        print("Problem with multiplexer: " + str(error))
    return multiplexer

def initialize_light2n3()->Optional[Bh1750]:
    set_multiplexer_channel(6)
   
    try:
        light2n3 = Bh1750()
        light2n3.luminance(Bh1750.ONCE_LOWRES)
    except Exception as error:
        light2n3 = None
        print("Problem with light sensor 2 or 3: " + str(error))
    return light2n3

while True:            
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
        
       # pidController = CircularPIDController(0.3,0,0,360)
        #calculatedAngle = pidController.calculate(goal_angle, motor.angle)
        #motor.move_angle(calculatedAngle)
        #print(calculatedAngle, goal_angle)
        motor.set_angle(goal_angle)
        print(goal_angle, motor.angle)

    except Exception as error:
        print("Error in rotation mechanism: " + str(error))
        
        # dont overload cpu
        sleep(0.1)
