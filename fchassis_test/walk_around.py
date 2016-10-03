#!/usr/bin/env python

import sys, time, json
from lib.frobo_ng import frobo_ng
import picamera
from lib.camera import update_img
from lib.utils import dbprint

c = frobo_ng()
c.debug = True
try:
	h = c.heading()
	try:
		c.update_state()
		dbprint('BEFORE %d (%d:%d) dist=%g(%g)' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], c.state.get('irdist', -1)))
		c.cmd_walk_around(70, 70)
		time.sleep(60)
		c.update_state()
		dbprint('AFTER slide %d (%d:%d) dist:%g(%g)' % (
				c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], c.state.get('irdist', -1)))

		json.dump(c.dots, file('dots.json', 'w'), indent=2)
	finally:
		c.cmd_mstop()
		c.update_state()
		change = c.heading() - h
		dbprint('EVENTUALLY %d (%d(%.2fm):%d(%.2fm)) dist=%g(%g), turn=%d' % (
			c.heading(), c.state['lcount'], c.steps2m(c.state['lcount']),
			c.state['rcount'], c.steps2m(c.state['rcount']), c.state['sonar'], c.state.get('irdist', -1), change))
finally:
#	cam = picamera.PiCamera()
	time.sleep(2)
#	update_img(cam)
