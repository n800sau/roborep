#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	clockwise = int(sys.argv[1])

	c = frobo_ng()
	c.debug = True

	with picamera.PiCamera() as camera:

		try:
			print 'BEFORE %s cm to %s' % (c.state['sonar'], c.heading())
			c.find_distance(1, clockwise=clockwise)
			print 'AFTER %s cm to %s' % (c.state['sonar'], c.heading())
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			update_img(camera)
