#!/usr/bin/env python

import sys, os, time, json, redis, picamera, cv2

from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo
from lib.marker import use_camera, release_camera, marker_offset

TARGET = 175

if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	r = redis.Redis()

	with frobo(s_port) as c:
		#c.debug = False

		off = None
		try:
			use_camera(r)
			time.sleep(4)
			for i in range(10):
				off = marker_offset(r, TARGET)
				if off is None:
					continue
				else:
					dbprint('H offset: %d degree%s'  % (abs(off), ((' in %s direction' % ('left' if off < 0 else 'right')) if int(off) != 0 else '')))
					if int(off) == 0:
						break
					if off < 0:
						c.tick_left()
					else:
						c.tick_right()
		finally:
			release_camera(r)
			time.sleep(1)
			c.stop()
			if off is None:
				camera = picamera.PiCamera()
				update_img(camera)
