#!/usr/bin/env python

import time, json
from lib.frobo_ng import frobo_ng
import picamera
from lib.camera import update_img
from lib.utils import dbprint

debug = False
c = frobo_ng(debug=debug)
try:
	dbprint('BEFORE %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))
	c.move_straight(fwd=True, max_steps=50, max_secs=1)
	dbprint('AFTER %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))

	json.dump(c.dots, file('dots.json', 'w'), indent=2)
finally:
	update_img(picamera.PiCamera())
	c.update_state()
	dbprint('EVENTUALLY %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))
