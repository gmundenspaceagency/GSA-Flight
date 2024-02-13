import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    print(GPIO.input(10))
