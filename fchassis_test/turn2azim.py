#!/usr/bin/env python

import sys, os, time

import picamera
from lib.hmc5883l import hmc5883l
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo

AZIM = 120

if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		c.debug = False

		with picamera.PiCamera() as camera:

			try:
				dbprint('BEFORE %d (%d:%d)' % (c.compass.heading(), c.mleft['count'], c.mright['count']))
				c.turn(AZIM)
				dbprint('AFTER %d (%d:%d)' % (c.compass.heading(), c.mleft['count'], c.mright['count']))
			finally:
				update_img(camera)
