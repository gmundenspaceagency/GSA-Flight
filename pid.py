
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
        print(error,"error")
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output
pidController = CircularPIDController(0.7,0.3,0.15,360)
i= 0
currentAngle=0
listOfCurrentAngles=[0,]
targetAngle=90
listOfTargetAngles=[targetAngle]
while i < 10:
    calculatedAngle = pidController.calculate(targetAngle, currentAngle)
    currentAngle = currentAngle+calculatedAngle
    listOfCurrentAngles.append(currentAngle)
    targetAngle=targetAngle-3
    listOfTargetAngles.append(targetAngle)
    i=i+1
    
print(listOfCurrentAngles)
