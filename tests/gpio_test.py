import RPi.GPIO as GPIO
from gpiozero import LED, Button
from time import sleep

led = LED(25)
led.on()
