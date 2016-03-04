#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint, html_data_path
from lib.camera import update_img

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	ad_data = []

	def cb(c):
		c.update_state()
		if c.state['sonar'] > 0:
			ad_data.append((c.heading(), c.state['sonar']))

	c = frobo_ng()
	c.debug = True

	t = time.time()
	vname = 'v.h264'
	with picamera.PiCamera() as camera:
		camera.resolution = (160, 120)
		camera.start_recording(vname, format='h264', quality=23)
		try:
			dbprint('BEFORE %d (%d:%d)' % (c.heading(), c.state['lcount'], c.state['rcount']))
			cnt = 8
			delta = -360 / cnt
			ar = [(v, v+45, 0) for v in range(0, 360, 45)]
			diff_acc = 0
			old_h = h = c.heading()
			for i in range(cnt):
				azim = h + delta
				dbprint('next azim=%d' % azim)
				c.simple_turn(azim=azim, pwr=40, cb_func=cb, precise_stop=False)
				h = c.heading()
				diff_acc += abs(old_h - h) % 360
				dbprint('diff_acc=%d' % diff_acc)
				if diff_acc > 360:
					break
			dbprint('AFTER %d (%d:%d)' % (c.heading(), c.state['lcount'], c.state['rcount']))
			c.wait_until_stop()
			json.dump(ad_data, file('ad_data.json', 'w'), indent=2)
		finally:
			c.cmd_mstop()
			camera.stop_recording()
			update_img(camera)
			dbprint('EVENTUALLY %d (%d:%d), dist:%g, dT:%d' % (c.heading(), c.state['lcount'], c.state['rcount'], c.state['sonar'], time.time()-t))

