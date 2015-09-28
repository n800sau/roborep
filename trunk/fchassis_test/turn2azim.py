#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	azim = float(sys.argv[1])

	debug = False
	c = frobo_ng(debug=debug)

	try:
		dbprint('BEFORE %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.state['lcount'], c.state['rcount'], azim))
		c.turn_in_ticks(azim, err=2)
		dbprint('AFTER %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.state['lcount'], c.state['rcount'], azim))
		time.sleep(3)
		dbprint('AFTER WAIT %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.state['lcount'], c.state['rcount'], azim))
		json.dump(c.dots, file('dots.json', 'w'), indent=2)
	finally:
		update_img(picamera.PiCamera())
