import time
from machine import Pin

class Motor:
    def __init__(self, in1, in2, step_count=200, step_sleep=1000):
        self.step_pin = Pin(in1, Pin.OUT)
        self.dir_pin = Pin(in2, Pin.OUT)
        self.step_count = step_count
        self.step_sleep = step_sleep
        self.motor_step_counter = 0
        self.current_angle = 0
        self.direction = True
        self.dir_pin.value(0)
        
    def move_steps(self, steps):
        if steps > 0:
            if not self.direction:
                self.dir_pin.value(0)
                self.direction = True
        else:
            if self.direction:
                self.dir_pin.value(1)
                self.direction = False
                
        for _ in range(abs(steps)):
            self.step_pin.toggle()
            time.sleep_us(self.step_sleep)
            self.step_pin.toggle()
            time.sleep_us(self.step_sleep)
            
        
    def move_angle(self, angle):
        if not abs(angle) > 1.8:
            return
        steps = int((angle / 360) * self.step_count)
        self.move_steps(steps)
        self.current_angle += angle
            
motor = Motor(17, 16, 200, 1000)
motor.move_angle(360)





        
        

        
        
