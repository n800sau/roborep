#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo

if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	distance = float(sys.argv[1])

	with frobo(s_port) as c:
#		c.debug = False

		with picamera.PiCamera() as camera:

			try:
				dbprint('BEFORE %d (%d:%d)' % (c.compass.heading(), c.mleft['count'], c.mright['count']))
				c.fwd_straight(max_secs=2, max_steps=c.m2steps(distance), power=100)
				dbprint('AFTER %d (%d:%d)' % (c.compass.heading(), c.mleft['count'], c.mright['count']))
				json.dump(c.dots, file('dots.json', 'w'), indent=2)
			finally:
				update_img(camera)