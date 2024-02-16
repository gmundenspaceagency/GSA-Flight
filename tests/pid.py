import sys
sys.path.append('..')
from gsa_components.motor import StepperMotor
from time import sleep
import random

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

if __name__ == "__main__":
	pidController = CircularPIDController(0.7,0.3,0.15,360)
	motor = StepperMotor(26,19,13,6,200,0.002)
	i= 0
	currentAngle=0
	listOfCurrentAngles=[0,]
	targetAngle=90
	listOfTargetAngles=[targetAngle]
	while i < 500:
	    calculatedAngle = pidController.calculate(targetAngle, currentAngle)
	    if abs(calculatedAngle) > 1.8:
	        
	        motor.move_angle(calculatedAngle)
	        currentAngle = currentAngle+calculatedAngle
	        listOfCurrentAngles.append(currentAngle)
	    targetAngle=targetAngle+random.randint(-10,10)
	    listOfTargetAngles.append(targetAngle)
	    i=i+1
	    sleep(0.1)
	    
	print(listOfCurrentAngles)
