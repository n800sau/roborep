#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	c = frobo_ng()
	c.debug = True

	t = time.time()
	try:
		dbprint('BEFORE %d (%d:%d)' % (c.heading(), c.state['lcount'], c.state['rcount']))
		data = c.collect_turn_data(cnt=30)
#		data = c.collect_turn_data(cnt=100, pwr=50, dT=1, clockwise=True)
		dbprint('AFTER %d (%d:%d)' % (c.heading(), c.state['lcount'], c.state['rcount']))
		c.wait_until_stop()
		json.dump(data, file('data.json', 'w'), indent=2)
	finally:
		c.cmd_mstop()
		update_img(picamera.PiCamera())
		dbprint('EVENTUALLY %d (%d:%d), dist:%g, dT:%d' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], time.time()-t))

