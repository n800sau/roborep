#!/usr/bin/env python

import sys, os, time, json

import picamera, cv2
from lib.utils import dbprint
from lib.camera import update_img, ImageSearch, capture_cvimage

from lib.frobo import frobo

i = 1
def stop_cb(c, so, camera):
	global i
	rs = False
	frame = capture_cvimage(camera)
	for n in so.image_names():
		fdata = so.find_image(n, frame=frame)
		if not fdata is None:
			dbprint('Found %s (%d)' % (n, i))
			rs = True
			break
	fname = os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % i)
	cv2.imwrite(fname, frame)
	i += 1
	return rs

if __name__ == '__main__':

	clockwise = int(sys.argv[1])

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		#c.debug = False

		with picamera.PiCamera() as camera:

			so = ImageSearch(camera)
			so.add_image('owl', 'images/1sm.png')
#			so.add_image('elephant', 'images/2sm.png')
#			so.add_image('cat', 'images/3sm.png')
#			so.add_image('goat', 'images/4sm.png')

			try:
				print 'BEFORE %s cm to %s' % (c.curr_dist, c.compass.heading())
				c.search_around(lambda c, so=so, camera=camera: stop_cb(c, so, camera), clockwise=clockwise)
				print 'AFTER %s cm to %s' % (c.curr_dist, c.compass.heading())
				json.dump(c.dots, file('dots.json', 'w'), indent=2)
			finally:
				update_img(camera)
