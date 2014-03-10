#!/usr/bin/env python

import sys
sys.path.append('../../bb')

import bb_gpio_lib as bb

import time
reset_pin = 'P9_15'
bb.pinMode(reset_pin, bb.OUT)
try:
	bb.digitalWrite(reset_pin, 0)
	time.sleep(0.1)
	bb.digitalWrite(reset_pin, 1)
finally:
	bb.releasePin(reset_pin)
