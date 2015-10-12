#!/usr/bin/env python

import sys, os, time

import picamera
from lib.utils import dbprint
from lib.camera import update_img
from lib.frobo_ng import frobo_ng

LEFT = int(sys.argv[1])

c = frobo_ng()
c.debug = True
try:
	h = c.compass.heading()
	dbprint('BEFORE %.2f (%d:%d)' % (h, c.state['lcount'], c.state['rcount']))
	proc = c.tick_left if LEFT else c.tick_right
	proc(pwr=100, min_angle=90)
	new_h = c.compass.heading()
	dbprint('AFTER %.2f (%d:%d) turned:%.2f' % (new_h, c.state['lcount'], c.state['rcount'], new_h - h))
finally:
#	update_img(picamera.PiCamera())
	pass
