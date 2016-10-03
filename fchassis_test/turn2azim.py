#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	azim = float(sys.argv[1])

	c = frobo_ng()
	c.debug = True

	cam = picamera.PiCamera()
	t = time.time()
	try:
		dbprint('BEFORE %d (%d:%d), TARGET: %d' % (c.heading(), c.state['lcount'], c.state['rcount'], azim))
		c.turn(azim, err=2)
		dbprint('AFTER %d (%d:%d), TARGET: %d' % (c.heading(), c.state['lcount'], c.state['rcount'], azim))
		c.wait_until_stop()
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
	finally:
		c.cmd_mstop()
		time.sleep(2)
		update_img(cam)
		dbprint('EVENTUALLY %d (%d:%d), TARGET: %d dist:%g, dT:%d' % (c.heading(), c.state['lcount'], c.state['rcount'], azim, c.state['sonar'], time.time()-t))

