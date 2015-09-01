#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.hmc5883l import hmc5883l
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo

if __name__ == '__main__':

	azim = float(sys.argv[1])

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		#c.debug = False

		with picamera.PiCamera() as camera:

			try:
				dbprint('BEFORE %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.mleft['count'], c.mright['count'], azim))
				c.turn(azim, err=2)
				dbprint('AFTER %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.mleft['count'], c.mright['count'], azim))
				time.sleep(3)
				dbprint('AFTER WAIT %d (%d:%d), TARGET: %d' % (c.compass.heading(), c.mleft['count'], c.mright['count'], azim))
				json.dump(c.dots, file('dots.json', 'w'), indent=2)
			finally:
				update_img(camera)
