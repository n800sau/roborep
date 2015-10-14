#!/usr/bin/env python

import sys, os, time, json

import picamera, cv2
from lib.utils import dbprint, html_path
from lib.camera import update_img, ShapeSearch, capture_cvimage

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	c = frobo_ng()
	#c.debug = True

	with picamera.PiCamera() as camera:

		ss = ShapeSearch(camera)
		try:
			data = ss.find_shapes()
			if data:
				cv2.imwrite(html_path('shapes.jpg'), data['frame'])
		finally:
			update_img(camera)
