#!/usr/bin/env python

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.OUT)
GPIO.output(7, False)
GPIO.output(7, True)
