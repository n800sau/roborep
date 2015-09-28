#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img, FeatureProcess, capture_cvimage

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	right = int(sys.argv[1])

	debug = False

	c = frobo_ng(debug=debug)

	with picamera.PiCamera() as camera:

		fp = FeatureProcess(camera)
		try:
			update_img(camera, 'pic0.jpg')
			h = c.compass.heading()
			c.turn_in_steps((h + (1 if right else -1) * 90) % 360)
			c.fwd_straight(max_secs=1, max_steps=c.m2steps(0.4))
			c.turn_in_steps(h)
			update_img(camera, 'pic1.jpg')
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			update_img(camera)
