#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img, FeatureProcess, capture_cvimage

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	c = frobo_ng()
	#c.debug = True

	with picamera.PiCamera() as camera:

		fp = FeatureProcess(camera)
		try:
			update_img(camera, 'pic0.jpg')
			c.turn(0)
			update_img(camera, 'pic1.jpg')
			fp.percent()
			c.turn(90)
			c.move_straight(fwd=True, max_secs=1, max_steps=100)
			c.turn(0)
			dbprint('Matches %s%%' % fp.percent())
			update_img(camera, 'pic2.jpg')
			c.turn(90)
			c.move_straight(fwd=True, max_secs=1, max_steps=100)
			c.turn(0)
			dbprint('Matches %s%%' % fp.percent())
			update_img(camera, 'pic3.jpg')
			c.turn(90)
			c.move_straight(fwd=True, max_secs=1, max_steps=100)
			c.turn(0)
			dbprint('Matches %s%%' % fp.percent())
			update_img(camera, 'pic4.jpg')
			c.turn(90)
			c.move_straight(fwd=True, max_secs=1, max_steps=100)
			c.turn(0)
			dbprint('Matches %s%%' % fp.percent())
			update_img(camera, 'pic5.jpg')
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			update_img(camera)
