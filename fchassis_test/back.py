#!/usr/bin/env python

import time, json, sys
from lib.frobo_ng import frobo_ng
import picamera
from lib.camera import update_img
from lib.utils import dbprint

dist = float(sys.argv[1])

c = frobo_ng()
c.debug = True
try:
	h = c.heading()
	dbprint('BEFORE %d (%d:%d)' % (h, c.state['lcount'], c.state['rcount']))
	c.move_straight(fwd=False, max_steps=c.m2steps(dist), max_secs=5)
	dbprint('AFTER %d (%d:%d)' % (c.heading(), c.state['lcount'], c.state['rcount']))
	c.update_state()
	dbprint('acclist 1: %s' % json.dumps(c.state['acclist'], indent=2))
	dbprint('x=%g-%g,y=%g-%g,z=%g-%g' % (
		min([e['x'] for e in c.state['acclist']]),
		max([e['x'] for e in c.state['acclist']]),
		min([e['y'] for e in c.state['acclist']]),
		max([e['y'] for e in c.state['acclist']]),
		min([e['z'] for e in c.state['acclist']]),
		max([e['z'] for e in c.state['acclist']])
	))
	json.dump(c.dots, file('dots.json', 'w'), indent=2)
finally:
	update_img(picamera.PiCamera())
	c.update_state()
	change = c.heading() - h
	dbprint('acclist 2: %s' % json.dumps(c.state['acclist'], indent=2))
	dbprint('x=%g-%g,y=%g-%g,z=%g-%g' % (
		min([e['x'] for e in c.state['acclist']]),
		max([e['x'] for e in c.state['acclist']]),
		min([e['y'] for e in c.state['acclist']]),
		max([e['y'] for e in c.state['acclist']]),
		min([e['z'] for e in c.state['acclist']]),
		max([e['z'] for e in c.state['acclist']])
	))
	dbprint('EVENTUALLY %d (%d:%d) dist=%g, turn=%d' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], change))
