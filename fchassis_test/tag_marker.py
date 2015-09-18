#!/usr/bin/env python

import sys, os, time, json, redis, picamera, cv2

from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo
from lib.marker import use_camera, release_camera, marker_offset

TARGET = 175

def tick_left(c):
	init_h = c.compass.heading()
	dbprint('init h: %d' % init_h)
	try:
		c.left_move(False, 35)
		c.right_move(True, 35)
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

def tick_right(c):
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

	r = redis.Redis()

	with frobo(s_port) as c:
		#c.debug = False

		marker = None
		try:
			use_camera(r)
			time.sleep(4)
			for i in range(10):
				off = marker_offset(r, TARGET)
				if off is None:
					break
				else:
					dbprint('H offset: %d degree%s'  % (abs(off), ((' in %s direction' % ('left' if off < 0 else 'right')) if int(off) != 0 else '')))
					if int(off) == 0:
						break
					if off < 0:
						tick_left(c)
					else:
						tick_right(c)
		finally:
			release_camera(r)
			time.sleep(1)
			c.stop()
			camera = picamera.PiCamera()
			update_img(camera)


