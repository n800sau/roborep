#!/usr/bin/env python3

#config-pin P9_13 gpio
#config-pin P9_14 gpio

import Adafruit_BBIO.GPIO as GPIO
import time, sys

#P8_13 -> i00
#P8_14 -> chip_en

# Set up pins as inputs or output
GPIO.setup("P8_13", GPIO.OUT)
GPIO.setup("P8_14", GPIO.OUT)
#GPIO.setup("GPIO0_26", GPIO.OUT)  # Alternative: use actual pin names

GPIO.output("P8_13", GPIO.LOW)  # You can also write '1' instead
GPIO.output("P8_14", GPIO.LOW)   # You can also write '0' instead
time.sleep(0.1)
GPIO.output("P8_14", GPIO.HIGH)   # You can also write '0' instead
