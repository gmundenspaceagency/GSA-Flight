#from pid import *
import RPi.GPIO as GPIO
import time

class Motor:
    def __init__(self, in1, in2, in3, in4, step_count=200, step_sleep=0.02):
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.step_count = step_count
        self.step_sleep = step_sleep
        self.direction = True  # True for clockwise, False for counter-clockwise
        self.step_sequence = [
            [1, 0, 0, 1],
            [0, 1, 0, 1],
            [0, 1, 1, 0],
            [1, 0, 1, 0]
        ]

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)

        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)

        self.motor_pins = [self.in1, self.in2, self.in3, self.in4]
        self.motor_step_counter = 0
        self.angle = 0

        def cleanup(self):
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.LOW)
            GPIO.output(self.in3, GPIO.LOW)
            GPIO.output(self.in4, GPIO.LOW)
            GPIO.cleanup()

    def move_steps(self, steps):
        self.moving = True
        for i in range(steps):
            
            if not self.moving:
                break
            
            for pin in range(0, len(self.motor_pins)):
                # has to be set again here because of weird GPIO error in second thread please FIX
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.in1, GPIO.OUT)
                GPIO.setup(self.in2, GPIO.OUT)
                GPIO.setup(self.in3, GPIO.OUT)
                GPIO.setup(self.in4, GPIO.OUT)
                GPIO.output(self.motor_pins[pin], self.step_sequence[self.motor_step_counter][pin])
            if self.direction:
                self.motor_step_counter = (self.motor_step_counter - 1) % 4
                self.angle -= 360 / self.step_count
                
                if self.angle < 0:
                    self.angle += 360
            else:
                self.motor_step_counter = (self.motor_step_counter + 1) % 4
                self.angle += 360 / self.step_count
                
                if self.angle > 360:
                    self.angle -= 360
                
            time.sleep(self.step_sleep)        

    def move_angle(self, angle, min_movement=10):
        steps = int((angle / 360) * self.step_count)
        self.direction = angle < 0
        self.move_steps(abs(steps))

    def set_angle(self, angle):
        move = angle - self.angle
        
        if move > 180:
            move = 360 - move
        if move < -180:
            move = 360 + move
        
        self.move_angle(move)
      
"""
pidController = CircularPIDController(0.7,0.3,0.15,360)
i= 0
currentAngle=0
listOfCurrentAngles=[0,]
targetAngle=90
listOfTargetAngles=[targetAngle]
smallErrorAddition = 0
while i < 400:
    calculatedAngle = pidController.calculate(targetAngle, currentAngle)
    if abs(calculatedAngle) > 2.8:
        print("angle bigger than 1", calculatedAngle)
        motor.move_angle(calculatedAngle)
        currentAngle = currentAngle+calculatedAngle
        listOfCurrentAngles.append(currentAngle)
    targetAngle=targetAngle-6
    listOfTargetAngles.append(targetAngle)
    i=i+1
    sleep(0.1)
    
    
print(listOfCurrentAngles)
"""
