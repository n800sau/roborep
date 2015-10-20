#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	clockwise = int(sys.argv[1])

	c = frobo_ng()
	c.debug = True

	t = time.time()
	try:
		dbprint('BEFORE %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))
		data = c.collect_turn_data(cnt=1000, dT=1)
		dbprint('AFTER %d (%d:%d)' % (c.compass.heading(), c.state['lcount'], c.state['rcount']))
		c.wait_until_stop()
		json.dump(data, file('data.json', 'w'), indent=2)
		df = file('data', 'w')
		print >>df, len(data), len(data[0]['input']), len(data[0]['output'])
		for d in data:
			for k in sorted(d['input'].keys()):
				print >>df, d['input'][k],
			print >>df
			for k in sorted(d['output'].keys()):
				print >>df, d['output'][k],
			print >>df
		df.close()
	finally:
		c.cmd_mstop()
		update_img(picamera.PiCamera())
		dbprint('EVENTUALLY %d (%d:%d), dist:%g, dT:%d' % (c.compass.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], time.time()-t))

