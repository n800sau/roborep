#!/usr/bin/env python

import sys, os, time, json

import matplotlib as mpl
mpl.use('Agg')
import picamera, cv2
from lib.utils import dbprint, html_data_path
from lib.camera import update_img, FeatureProcess

clrLower = [0, 10, 10]
clrUpper = [64, 255, 255]

if __name__ == '__main__':

	with picamera.PiCamera() as camera:

		ss = FeatureProcess(camera)
		try:
			data = ss.mask_range(clrLower, clrUpper)
				if data:
#					cv2.imwrite(html_data_path('frame_%03d.jpg' % i), data['frame'])
#					cv2.imwrite(html_data_path('iframe_%03d.jpg' % i), data['iframe'])
#					cv2.imwrite(html_data_path('oframe_%03d.jpg' % i), data['oframe'])
					cv2.imwrite(html_data_path('frame.jpg'), data['frame'])
					cv2.imwrite(html_data_path('iframe.jpg'), data['iframe'])
					cv2.imwrite(html_data_path('oframe.jpg'), data['oframe'])
#					json.dump(data['hlist'], file('colorfix.json', 'w'))
				else:
					dbprint("NOT FOUND")
#			json.dump({'imgcount': i}, file(html_data_path('frames.json'), 'w'), indent=2)
		finally:
			update_img(camera)
