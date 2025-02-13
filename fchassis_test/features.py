#!/usr/bin/env python

import sys, os, time, json

import matplotlib as mpl
mpl.use('Agg')
import picamera, cv2, traceback
from lib.utils import dbprint, html_data_path
from lib.camera import update_img, FeatureProcess

from lib.frobo_ng import frobo_ng

if __name__ == '__main__':

	c = frobo_ng()
	with picamera.PiCamera() as camera:

		ss = FeatureProcess(camera)
		try:
			ss.percent()
			c.tick_left(min_angle=6)
			data = ss.percent()
			if data:
#				cv2.imwrite(html_data_path('frame_%03d.jpg' % i), data['frame'])
#				cv2.imwrite(html_data_path('iframe_%03d.jpg' % i), data['iframe'])
#				cv2.imwrite(html_data_path('oframe_%03d.jpg' % i), data['oframe'])
				cv2.imwrite(html_data_path('frame.jpg'), data['frame'])
				cv2.imwrite(html_data_path('iframe.jpg'), data['iframe'])
				cv2.imwrite(html_data_path('oframe.jpg'), data['mframe'])
#				json.dump(data['hlist'], file('colorfix.json', 'w'))
			else:
				dbprint("NOT FOUND")
#			json.dump({'imgcount': i}, file(html_data_path('frames.json'), 'w'), indent=2)
#		except:
#			traceback.print_exc()
		finally:
			update_img(camera)
