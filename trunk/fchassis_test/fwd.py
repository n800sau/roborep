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
	c.update_state()
	dbprint('BEFORE %d (%d:%d) dist=%g(%g)' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], c.state.get('irdist', -1)))
	c.move_straight(fwd=True, max_steps=c.m2steps(dist), max_secs=10, power=90)
	dbprint('AFTER %d (%d:%d) dist=%g(%g)' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], c.state.get('irdist', -1)))
	if not c.hit_warn is None:
		dbprint("Slide back")
		c.move_straight(fwd=False, max_steps=c.m2steps(0.2), max_secs=2)
		dbprint('AFTER slide %d (%d:%d) dist:%g(%g)' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], c.state.get('irdist', -1)))

	json.dump(c.dots, file('dots.json', 'w'), indent=2)
finally:
	c.cmd_mstop()
	cam = picamera.PiCamera()
	c.update_state()
	change = c.heading() - h
	time.sleep(2)
	update_img(cam)
	dbprint('EVENTUALLY %d (%d:%d) dist=%g(%g), turn=%d' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], c.state.get('irdist', -1), change))
