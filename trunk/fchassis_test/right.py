#!/usr/bin/env python

import sys, os, time

import picamera
from lib.hmc5883l import hmc5883l
from lib.utils import dbprint
from lib.camera import update_img

from lib.fchassis import fchassis

def right(c):
	compass = hmc5883l(gauss = 4.7, declination = (12, 34))
	dbprint('%s' % (compass.degrees(compass.heading())[0]))
	try:
		c.left_move(True, 50)
		c.right_move(False, 50)
		time.sleep(0.3)
	finally:
		c.stop()
		dbprint('%s' % (compass.degrees(compass.heading())[0]))


if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with fchassis(s_port) as c:
		c.debug = False

		with picamera.PiCamera() as camera:

			try:
				right(c)
			finally:
				update_img(camera)