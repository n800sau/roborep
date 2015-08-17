#!/usr/bin/env python

import sys, os, time

import picamera
from lib.hmc5883l import hmc5883l
from lib.utils import dbprint
from lib.camera import update_img

from lib.fchassis import fchassis

AZIM = 90
ERR = 10

# angle difference in degrees
def angle_diff(a1, a2):
	return ((a1 - a2) + 180) % 360 - 180

def turn(c, azim):
	compass = hmc5883l(gauss = 4.7, declination = (12, 34))
	dbprint('%s' % (compass.degrees(compass.heading())[0]))
	try:
		heading = compass.degrees(compass.heading())[0]
		diff = angle_diff(azim, heading)
		if abs(diff) > ERR:
			if diff > 0:
				c.left_move(True, 50)
				c.right_move(False, 50)
			else:
				c.left_move(False, 50)
				c.right_move(True, 50)
			for i in range(100):
				heading = compass.degrees(compass.heading())[0]
				diff = abs(angle_diff(azim, heading))
				dbprint("diff=%d" % diff)
				if diff < ERR:
					break
				time.sleep(0.01)
	finally:
		c.stop()
		dbprint('%s' % (compass.degrees(compass.heading())[0]))


if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with fchassis(s_port) as c:
		c.debug = False

		with picamera.PiCamera() as camera:

			try:
				turn(c, AZIM)
			finally:
				update_img(camera)
