#!/usr/bin/env python

import sys, os, time, json

import picamera, cv2
from lib.utils import dbprint
from lib.camera import update_img, ImageSearch, capture_cvimage

from lib.frobo import frobo

i = 0
def stop_cb(c, so, camera):
	global i
	rs = False
#	frame = capture_cvimage(camera)
	frame = capture_cvimage(camera, resolution=(1280, 960))
#	frame = capture_cvimage(camera, resolution=(2592,1944))
	for n in so.image_names():
		fdata = so.find_image(n, frame=frame)
		if not fdata is None:
			dbprint('Found %s (%d) at %g (%s)' % (n, i, c.compass.heading(), fdata['point']))
#			rs = True
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
			data = (
				('a', 'images/a.png'),
				('b', 'images/b.png'),
				('c', 'images/c.png'),
				('d', 'images/d.png'),
				('f', 'images/f.png'),
				('g', 'images/g.png'),
			)

			try:
				c.update_dist()
				dbprint('BEFORE %s cm to %s' % (c.curr_dist, c.compass.heading()))
				c.search_around(lambda c, so=so, camera=camera: stop_cb(c, so, camera), clockwise=clockwise)
				dbprint('AFTER %s cm to %s (%d steps)' % (c.curr_dist, c.compass.heading(), i))
				json.dump(c.dots, file('dots.json', 'w'), indent=2)
			finally:
				c.stop()
				time.sleep(1)
				update_img(camera)