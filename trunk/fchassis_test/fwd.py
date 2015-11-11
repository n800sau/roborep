#!/usr/bin/env python

import sys, time, json
from lib.frobo_ng import frobo_ng
import picamera
from lib.camera import update_img
from lib.utils import dbprint

dist = float(sys.argv[1])

c = frobo_ng()
c.debug = True
try:
	h = c.heading()
	dbprint('BEFORE %d (%d:%d) dist=%g' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar']))
	c.move_straight(fwd=True, max_steps=c.m2steps(dist), max_secs=20, power=80)
	dbprint('AFTER %d (%d:%d) dist=%g' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar']))
	if not c.hit_warn is None:
		dbprint("Slide back")
		c.move_straight(fwd=False, max_steps=c.m2steps(0.2), max_secs=5)
		dbprint('AFTER slide %d (%d:%d) dist:%g' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar']))

	json.dump(c.dots, file('dots.json', 'w'), indent=2)
finally:
	c.cmd_mstop()
	update_img(picamera.PiCamera())
	c.update_state()
	change = c.heading() - h
	dbprint('EVENTUALLY %d (%d:%d) dist=%g, turn=%d' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], change))
