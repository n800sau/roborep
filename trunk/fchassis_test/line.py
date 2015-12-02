#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint, html_data_path
from lib.camera import update_img, FeatureProcess, capture_cvimage

from lib.frobo_ng import frobo_ng

AZIM_SHOT = 350
AZIM_MOVE = 90

if __name__ == '__main__':

	c = frobo_ng()
	c.debug = True

	with picamera.PiCamera() as camera:

		fp = FeatureProcess(camera)
		try:
			update_img(camera, html_data_path('pic0.jpg'))
			c.turn(AZIM_SHOT)
			update_img(camera, html_data_path('pic1.jpg'))
			fp.percent()
			c.turn(AZIM_MOVE)
			c.move_straight(fwd=True, max_secs=1, max_steps=100)
			c.turn(AZIM_SHOT)
			dbprint('Matches %s%%' % fp.percent())
			update_img(camera, html_data_path('pic2.jpg'))
			c.turn(AZIM_MOVE)
			c.move_straight(fwd=True, max_secs=1, max_steps=100)
			c.turn(AZIM_SHOT)
			dbprint('Matches %s%%' % fp.percent())
			update_img(camera, html_data_path('pic3.jpg'))
			c.turn(AZIM_MOVE)
			c.move_straight(fwd=True, max_secs=1, max_steps=100)
			c.turn(AZIM_SHOT)
			dbprint('Matches %s%%' % fp.percent())
			update_img(camera, html_data_path('pic4.jpg'))
			c.turn(AZIM_MOVE)
			c.move_straight(fwd=True, max_secs=1, max_steps=100)
			c.turn(AZIM_SHOT)
			dbprint('Matches %s%%' % fp.percent())
			update_img(camera, html_data_path('pic5.jpg'))
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			update_img(camera)
