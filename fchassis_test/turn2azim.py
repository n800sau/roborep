#!/usr/bin/env python

import sys, os, time, json

import picamera, cv2
from lib.hmc5883l import hmc5883l
from lib.utils import dbprint
from lib.camera import update_img, ImageSearch

from lib.frobo import frobo

i = 0
def turn_cb(c, so):
	global i
	fdata = so.find_image('goat')
	if not fdata is None:
		fname = os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % i)
		cv2.imwrite(fname, fdata['frame'])
		i += 1

if __name__ == '__main__':

	azim = float(sys.argv[1])

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		#c.debug = False

		with picamera.PiCamera() as camera:

			so = ImageSearch(camera)
			so.add_image('owl', 'images/1sm.png')
			so.add_image('elephant', 'images/2sm.png')
			so.add_image('cat', 'images/3sm.png')
			so.add_image('goat', 'images/4sm.png')
			try:
				dbprint('BEFORE %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.mleft['count'], c.mright['count'], azim))
				c.turn(azim, err=2, move_cb=lambda c, so=so: turn_cb(c, so))
				dbprint('AFTER %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.mleft['count'], c.mright['count'], azim))
				time.sleep(3)
				dbprint('AFTER WAIT %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.mleft['count'], c.mright['count'], azim))
				json.dump(c.dots, file('dots.json', 'w'), indent=2)
			finally:
				update_img(camera)
