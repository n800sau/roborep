#!/usr/bin/env python

import sys, os, time

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo

def right(c):
	init_h = c.compass.heading()
	dbprint('init h: %d' % init_h)
	try:
		c.left_move(True, 35)
		c.right_move(False, 35)
		for i in range(100):
			st = c.gyro.bus.read_byte_data(c.gyro.address, c.gyro.STATUS_REG)
			if st:
				x,y,z = c.gyro.getDegPerSecAxes()
				h = c.compass.heading()
				dbprint('g:%d %d %d, h:%d' % (x, y, z, h))
				if abs(z) > 15:
					dbprint('it moves')
					break
			time.sleep(0.001)
	finally:
		c.stop()
		time.sleep(2)
		h = c.compass.heading()
		dbprint('last h: %d, change: %g' % (h, init_h - h))


if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		c.debug = False

		with picamera.PiCamera() as camera:

			try:
				right(c)
			finally:
				update_img(camera)
