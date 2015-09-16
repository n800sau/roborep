#!/usr/bin/env python

import sys, os
import time

import picamera, cv2
from lib.camera import update_img, ImageSearch, capture_cvimage
from lib.utils import dbprint

if __name__ == '__main__':

	with picamera.PiCamera() as camera:

		data = (
#			('a', 'images/a.png'),
#			('b', 'images/b.png'),
#			('c', 'images/c.png'),
#			('d', 'images/d.png'),
#			('f', 'images/f.png'),
#			('g', 'images/g.png'),
		)
		time.sleep(1)
		so = ImageSearch(camera)
		frame = capture_cvimage(camera, resolution=(2592,1944))
#		frame = capture_cvimage(camera, resolution=(1280, 960))
#		frame = capture_cvimage(camera, resolution=(640, 480))

		i = 0
		for d in data:
			so.add_image(*d)

			fname = os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % i)
			fdata = so.find_image(d[0], frame=frame.copy())
#			if fdata is None:
#				dbprint('tot shape=%s, %s' % (frame.shape, frame.dtype))
#				n = 6
#				dw = frame.shape[1] / n
#				dh = frame.shape[0] / n
#				for i in range(n-1):
#					for j in range(n-1):
#						x0 = i*dw
#						y0 = j*dh
#						x1 = (i+2)*dw
#						y1 = (j+2)*dh
#						f = frame[y0: y1, x0: x1]
#						dbprint('%d,%d rect=%s shape=%s' % (i, j, (x0, y0, x1, y1), f.shape))
#						fdata = so.find_image(d[0], frame=f)
#						if not fdata is None:
#							break
#					if not fdata is None:
#						break
			if fdata is None:
				if os.path.exists(fname):
					os.unlink(fname)
			else:
				dbprint('Found %s, %s' % (d[0], fdata['frame'].shape))
				small_frame = cv2.resize(fdata['frame'], (640, 480))
				cv2.imwrite(fname, small_frame)
				del fdata
				i += 1

		update_img(camera)
