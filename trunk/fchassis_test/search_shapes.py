#!/usr/bin/env python

import sys, os, time, json

from lib.utils import dbprint
from lib.frobo_ng import frobo_ng
from lib.camera import update_img
import picamera, cv2

if __name__ == '__main__':

	clockwise = int(sys.argv[1])

	with picamera.PiCamera() as camera:

		c = frobo_ng()
		c.debug = True

		try:
			c.update_state()
			dbprint('BEFORE %s m to %s' % (c.state['sonar'], c.compass.heading()))
			c.search_shapes(camera, clockwise=clockwise)
			dbprint('AFTER %s m to %s' % (c.state['sonar'], c.compass.heading()))
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			c.cmd_mstop()
			c.wait_until_stop()
			update_img(camera)
