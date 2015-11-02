#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import html_data_path
from lib.utils import dbprint
from lib.camera import update_img, FeatureProcess

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	c = frobo_ng()
	#c.debug = True

	with picamera.PiCamera() as camera:

		fp = FeatureProcess(camera)
		try:
			update_img(camera, html_data_path('pic0.jpg'))
			c.turn(350)
			time.sleep(1)
			update_img(camera, 'images/pic1.jpg'))
			fp.percent()
			c.move_straight(fwd=True, max_steps=c.m2steps(0.5), max_secs=1)
			update_img(camera, html_data_path('pic2.jpg'))
			time.sleep(1)
			c.turn(90)
			time.sleep(1)
			c.move_straight(fwd=True, max_steps=c.m2steps(0.5), max_secs=1)
			update_img(camera, html_data_path('pic3.jpg'))
			time.sleep(1)
			c.turn(210)
			time.sleep(1)
			c.move_straight(fwd=True, max_steps=c.m2steps(0.5), max_secs=1)
			update_img(camera, html_data_path('pic4.jpg'))
			time.sleep(1)
			c.turn(270)
			time.sleep(1)
			c.move_straight(fwd=True, max_steps=c.m2steps(0.5), max_secs=1)
			update_img(camera, html_data_path('pic5.jpg'))
			dbprint('Matches %s%%' % fp.percent())
			json.dump(c.dots, file(html_data_path('dots.json'), 'w'), indent=2)
		finally:
			update_img(camera)
