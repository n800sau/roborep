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

	t = time.time()
	try:
		dbprint('BEFORE %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.state['lcount'], c.state['rcount'], azim))
		c.turn_in_ticks(azim, err=2)
		dbprint('AFTER %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.state['lcount'], c.state['rcount'], azim))
		c.wait_until_stop()
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
	finally:
		c.cmd_mstop()
		update_img(picamera.PiCamera())
		dbprint('EVENTUALLY %d (%d:%d), TARGET: %d dist:%g, dT:%d' % (c.compass.heading(), c.state['lcount'], c.state['rcount'], azim, c.state['sonar'], time.time()-t))
