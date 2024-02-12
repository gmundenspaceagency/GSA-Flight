import RPi.GPIO as GPIO  
import time 

class StepperMotor:

    def __init__(self, step_pin, dir_pin, step_count=200, step_delay=0.001):
        # Set up GPIO pins
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setup(step_pin, GPIO.OUT)
        GPIO.setup(dir_pin, GPIO.OUT)

        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.step_count = step_count
        self.step_delay = step_delay
        self.current_angle = 0  # Track current angle (0-360 degrees)

    def move_steps(self, steps):
        if steps == 0:
            return  # No need to move if steps is 0

        direction = steps > 0
        GPIO.output(self.dir_pin, direction)  # Set direction

        steps = abs(steps)  # Handle both positive and negative steps

        for _ in range(steps):
            GPIO.output(self.step_pin, True)  # Send step pulse
            time.sleep(self.step_delay)
            GPIO.output(self.step_pin, False)  # Release step pulse
            time.sleep(self.step_delay)

        self.current_angle += steps * (360 / self.step_count) * (1 if direction else -1)  # Update current angle

    def move_angle(self, angle):
        if abs(angle) > 360:  # Limit angle to 0-360 degrees
            angle = angle % 360

        steps = round(angle / 360 * self.step_count)  # Convert angle to steps
        self.move_steps(steps)

    def set_angle(self, angle):
        target_angle = angle % 360  # Wrap angle to 0-360
        steps = int((target_angle - self.current_angle) / 360 * self.step_count)
        self.move_steps(steps)
        self.current_angle = target_angle  # Update current angle

    def cleanup(self):
        GPIO.cleanup()