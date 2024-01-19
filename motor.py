"""
import RPi.GPIO as GPIO
import time

    class MOTOR:
        def __init__(self, in1, in2, in3, in4, step_count=4096, step_sleep=0.0008):
            self.in1 = in1
            self.in2 = in2
            self.in3 = in3
            self.in4 = in4
            self.step_count = step_count
            self.step_sleep = step_sleep
            self.direction = True  # True for clockwise, False for counter-clockwise
            self.step_sequence = [
                [1, 0, 0, 0],
                [1, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 1, 0],
                [0, 0, 1, 0],
                [0, 0, 1, 1],
                [0, 0, 0, 1],
                [1, 0, 0, 1]
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
                    GPIO.output(self.motor_pins[pin], self.step_sequence[self.motor_step_counter][pin])
                if self.direction:
                    self.motor_step_counter = (self.motor_step_counter - 1) % 8
                    self.angle -= 360 / self.step_count
                    
                    if self.angle < 0:
                        self.angle += 360
                else:
                    self.motor_step_counter = (self.motor_step_counter + 1) % 8
                    self.angle += 360 / self.step_count
                    
                    if self.angle > 360:
                        self.angle -= 360
                    
                time.sleep(self.step_sleep)        

        def move_angle(self, angle, min_movement=10):
            if abs(angle) < min_movement:
                return
            
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
import time
from machine import Pin
from time import sleep

class MOTOR:
    def __init__(self, in1, in2, in3, in4, step_count=200, step_sleep=0.022):
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.in3 = Pin(in3, Pin.OUT)
        self.in4 = Pin(in4, Pin.OUT)
        self.step_count = step_count
        self.step_sleep = step_sleep
        self.direction = True  # True for clockwise, False for counter-clockwise
        self.step_sequence = [
            [1, 0, 0, 1],
            [0, 1, 0, 1],
            [0, 1, 1, 0],
            [1, 0, 1, 0]
        ]

        self.motor_pins = [self.in1, self.in2, self.in3, self.in4]
        self.motor_step_counter = 0
        self.angle = 0

    def cleanup(self):
        for pin in self.motor_pins:
            pin.value(0)

    def move_steps(self, steps):
        self.moving = True
        for i in range(steps):
            if not self.moving:
                break
            for pin in range(0, len(self.motor_pins)):
                self.motor_pins[pin].value(self.step_sequence[self.motor_step_counter][pin])
            if self.direction:
                self.motor_step_counter = (self.motor_step_counter - 1) % 4
                self.angle -= 360 / (self.step_count / 4)
                if self.angle < 0:
                    self.angle += 360
            else:
                self.motor_step_counter = (self.motor_step_counter + 1) % 4
                self.angle += 360 / (self.step_count / 4)
                if self.angle > 360:
                    self.angle -= 360
            time.sleep(self.step_sleep)

    def move_angle(self, angle, min_movement=10):
        steps = int((angle / 360) * (self.step_count))
        print(steps, "steps")
        self.direction = angle < 0
        self.move_steps(abs(steps))

    def set_angle(self, angle):
        move = angle - self.angle
        if move > 180:
            move = 360 - move
        if move < -180:
            move = 360 + move
        self.move_angle(move)


class CircularPIDController:
    def __init__(self, Kp, Ki, Kd, angle_range):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.angle_range = angle_range

    def calculate(self, target_angle, current_angle):
        error = (target_angle - current_angle + self.angle_range/2) % self.angle_range - self.angle_range/2
        self.integral += error

        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output
motor = MOTOR(0,1,2,3,200,0.003)


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


