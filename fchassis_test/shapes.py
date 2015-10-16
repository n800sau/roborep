#!/usr/bin/env python

import sys, os, time, json

import picamera, cv2
from lib.utils import dbprint, html_path
from lib.camera import update_img, ShapeSearch, capture_cvimage

if __name__ == '__main__':

	with picamera.PiCamera() as camera:

		ss = ShapeSearch(camera)
		try:
			data = ss.find_shapes()
			if data:
				if os.environ.get('RASPICAM_ROTATE', ''):
					angle = int(os.environ['RASPICAM_ROTATE'])
					rows,cols,depth = data['frame'].shape
					M = cv2.getRotationMatrix2D((cols/2,rows/2), 180, 1)
					data['frame'] = cv2.warpAffine(data['frame'], M, (cols,rows))
				cv2.imwrite(html_path('shapes.jpg'), data['frame'])
				cv2.imwrite(html_path('thresh.jpg'), data['thresh'])
		finally:
			update_img(camera)
