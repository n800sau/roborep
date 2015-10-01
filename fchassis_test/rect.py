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
			c.turn(350)
			time.sleep(1)
			update_img(camera, 'pic1.jpg')
			fp.percent()
			c.move_straight(fwd=True, max_steps=c.m2steps(0.5), max_secs=1)
			update_img(camera, 'pic2.jpg')
			time.sleep(1)
			c.turn(90)
			time.sleep(1)
			c.move_straight(fwd=True, max_steps=c.m2steps(0.5), max_secs=1)
			update_img(camera, 'pic3.jpg')
			time.sleep(1)
			c.turn(210)
			time.sleep(1)
			c.move_straight(fwd=True, max_steps=c.m2steps(0.5), max_secs=1)
			update_img(camera, 'pic4.jpg')
			time.sleep(1)
			c.turn(270)
			time.sleep(1)
			c.move_straight(fwd=True, max_steps=c.m2steps(0.5), max_secs=1)
			update_img(camera, 'pic5.jpg')
			dbprint('Matches %s%%' % fp.percent())
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			update_img(camera)
