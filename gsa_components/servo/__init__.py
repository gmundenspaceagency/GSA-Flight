import RPi.GPIO as GPIO
from time import sleep

class Servo:
    def __init__(self, pin_nr=24):
        self.pin_nr = pin_nr
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_nr, GPIO.OUT)
        self.pin = GPIO.PWM(self.pin_nr, 50)
        
    def set_angle(self, angle):
        self.pin.start(0)
        duty_cycle = angle / 18 + 2.5
        self.pin.ChangeDutyCycle(duty_cycle)
        self.angle = angle
        sleep(1)
        self.pin.stop()
