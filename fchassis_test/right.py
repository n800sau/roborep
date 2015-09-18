#!/usr/bin/env python

import sys, os, time

import picamera
from lib.hmc5883l import hmc5883l
from lib.l3g4200d import l3g4200
from lib.utils import dbprint
from lib.camera import update_img

from lib.fchassis import fchassis

def right(c):
	compass = hmc5883l(gauss = 4.7, declination = (12, 34))
	init_h = compass.heading()
	gyro = l3g4200(1)
	dbprint('init h: %d' % init_h)
	try:
		c.left_move(True, 35)
		c.right_move(False, 35)
		for i in range(100):
			st = gyro.bus.read_byte_data(gyro.address, gyro.STATUS_REG)
			if st:
				x,y,z = gyro.getDegPerSecAxes()
				h = compass.heading()
				dbprint('g:%d %d %d, h:%d' % (x, y, z, h))
				if abs(z) > 15:
					dbprint('it moves')
					break
			time.sleep(0.001)
	finally:
		c.stop()
		time.sleep(2)
		h = compass.heading()
		dbprint('last h: %d, change: %g' % (h, init_h - h))


if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with fchassis(s_port) as c:
		c.debug = False

		with picamera.PiCamera() as camera:

			try:
				right(c)
			finally:
				update_img(camera)
