#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img, FeatureProcess, capture_cvimage

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	right = int(sys.argv[1])

	c = frobo_ng()
	c.debug = True

	with picamera.PiCamera() as camera:

		fp = FeatureProcess(camera)
		h = c.compass.heading()
		try:
			update_img(camera, 'pic0.jpg')
			h = c.compass.heading()
			c.turn_in_ticks((h + (1 if right else -1) * 90) % 360, err=2)
			c.move_straight(fwd=True, max_secs=5, max_steps=c.m2steps(0.5))
			c.turn_in_ticks(h)
			update_img(camera, 'pic1.jpg')
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			c.cmd_mstop()
			c.update_state()
			update_img(camera)
			change = c.compass.heading() - h
			dbprint('EVENTUALLY %d dist=%g, turn=%g' % (c.compass.heading(), c.state['sonar'], change))
