#!/usr/bin/env python

import sys, time, json
from lib.frobo_ng import frobo_ng
import picamera
from lib.camera import update_img
from lib.utils import dbprint

dist = float(sys.argv[1])

debug = False
c = frobo_ng(debug=debug)
cam = picamera.PiCamera()
try:
	azim = c.compass.heading()
	dbprint('BEFORE %d (%d:%d)' % (azim, c.state['lcount'], c.state['rcount']))
	c.move_straight(fwd=True, max_steps=c.m2steps(dist), max_secs=1)
	dbprint('AFTER FWD %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))
	update_img(cam)
	azim -= 180
	dbprint("Turning around to %d" % azim)
	c.turn_in_ticks(azim, err=5)
	dbprint('AFTER TURN %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))

	json.dump(c.dots, file('dots.json', 'w'), indent=2)
finally:
	update_img(cam)
	c.update_state()
	dbprint('EVENTUALLY %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))