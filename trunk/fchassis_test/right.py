#!/usr/bin/env python

import sys, os, time

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo_ng import frobo_ng


try:
	debug = True
	c = frobo_ng(debug=debug)
	dbprint('BEFORE %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))
	c.tick_right()
	dbprint('AFTER %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))
finally:
	update_img(picamera.PiCamera())
