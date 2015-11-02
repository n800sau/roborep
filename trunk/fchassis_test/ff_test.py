#!/usr/bin/env python

import sys, os, time, json

import picamera, cv2
from lib.utils import dbprint, html_data_path
from lib.camera import update_img, FeatureProcess, capture_cvimage

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	c = frobo_ng()
#	c.debug = True

	with picamera.PiCamera() as camera:

		fp = FeatureProcess(camera)
		try:
			fp.percent()
			c.tick_left(min_angle=5)
			data = fp.percent()
			if data:
				cv2.imwrite(html_data_path('pic1.jpg'), data['frame'])
			dbprint('Left=%g' % (data['percent'] if data else data))
			cv2.imwrite(html_data_path('frame.jpg'), data['frame'])
			c.tick_right(min_angle=5)
			data = fp.percent()
			if data:
				cv2.imwrite(html_data_path('pic2.jpg'), data['frame'])
			dbprint('Right=%g' % (data['percent'] if data else data))
		finally:
			update_img(camera)
