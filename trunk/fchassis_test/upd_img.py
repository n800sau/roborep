#!/usr/bin/env python

import sys, os
import time

import picamera, cv2
from lib.camera import update_img, ImageSearch
from lib.utils import dbprint

if __name__ == '__main__':

	with picamera.PiCamera() as camera:

		data = (
			('owl', 'images/1sm.png'),
			('elephant', 'images/2sm.png'),
			('cat', 'images/3sm.png'),
			('goat', 'images/4sm.png'),
		)
		time.sleep(1)
		so = ImageSearch(camera)

		i = 1
		for d in data:
			so.add_image(*d)

			fname = os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % i)
			fdata = so.find_image(d[0])
			if fdata is None:
				if os.path.exists(fname):
					os.unlink(fname)
			else:
				dbprint('Found %s' % d[0])
				cv2.imwrite(fname, fdata['frame'])
				i += 1

		update_img(camera)
