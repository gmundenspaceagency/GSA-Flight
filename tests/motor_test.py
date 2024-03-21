from gsa_components import Motor
from time import sleep

motor = Motor(17, 27, 22)

for i in range(4):
    motor.move_angle(90)
    print("Current angle:", motor.current_angle)
    sleep(1)

