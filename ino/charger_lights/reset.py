#!/usr/bin/env python

import sys
sys.path.append('../../bb')

import bb_gpio_lib as bb

import time
reset_pin = 'P8_12'
bb.pinMode(reset_pin, bb.OUT)
try:
	bb.digitalWrite(reset_pin, 0)
	time.sleep(10)
	bb.digitalWrite(reset_pin, 1)
finally:
	bb.releasePin(reset_pin)
