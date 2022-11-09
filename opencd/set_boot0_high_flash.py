#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

BOOT0_PIN = 40
RESET_PIN = 12

# for GPIO numbering, choose BCM
#GPIO.setmode(GPIO.BCM)

# or, for pin numbering, choose BOARD  
GPIO.setmode(GPIO.BOARD)

GPIO.setup(BOOT0_PIN, GPIO.OUT)
#reset
GPIO.setup(RESET_PIN, GPIO.OUT)

# flash firmware
GPIO.output(BOOT0_PIN, 1)
GPIO.output(RESET_PIN, 0)
time.sleep(0.2)
GPIO.output(RESET_PIN, 1)

print('Finished')
