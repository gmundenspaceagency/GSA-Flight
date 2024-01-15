from machine import Pin
import random
import time

def turn_off(pins:list[Pin])->None:
    for pin in pins:
        pin.value(0)

def turn_on(pins:list[Pin])->None:
    for pin in pins:
        pin.value(1)

def rain(pins:list[Pin], wait:int, reps:int)->None:
    turn_off(pins)
    
    for _ in range(0, reps):
        for i, pin in enumerate(pins):
            pin.value(1)
            
            if i != 0:
                pins[i - 1].value(0)
            else:
                pins[-1].value(0)
                
            time.sleep(wait)
            
def blink(pins:list[Pin], wait:int, reps:int)->None:
    for _ in range (0, reps):
        turn_off(pins)
        time.sleep(wait)
        turn_on(pins)
        time.sleep(wait)
            
def bounce(pins:list[Pin], wait:int, reps:int)->None:
    turn_off(pins)
    
    for _ in range(0, reps):
        for i, pin in enumerate(pins):
            pin.value(1)
            
            if i != 0:
                pins[i - 1].value(0)
            else:
                pins[-1].value(0)
                
            time.sleep(wait)
            
        for i in range(1, len(pins)):
            pin = pins[-i]
            pin.value(1)
            
            if i != 1:
                pins[-i + 1].value(0)
            else:
                pins[0].value(0)
                
            time.sleep(wait)

def main()->None:
    pins = []
    button1 = Pin(26, Pin.IN, Pin.PULL_UP)
    button2 = Pin(27, Pin.IN, Pin.PULL_UP)
    button3 = Pin(28, Pin.IN, Pin.PULL_UP)
    
    for i in range(0, 5):
        pins.append(Pin(i, Pin.OUT))
    
    try:
        while True:
            if button1.value() == 0: rain(pins, 0.1, 1)
            bounce(pins, 0.05, 1)
            if button3.value() == 0: blink(pins, 0., 1)
    finally:
        turn_off(pins)

main()
