#!/usr/bin/env python

import sys, os, time

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo

if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		#c.debug = False

		with picamera.PiCamera() as camera:

			try:
				c.turn(0)
				c.fwd_straightly(max_secs=1, max_steps=20)
				update_img(camera)
				c.turn(90)
				c.fwd_straightly(max_secs=1, max_steps=20)
				update_img(camera)
				c.turn(180)
				c.fwd_straightly(max_secs=1, max_steps=20)
				update_img(camera)
				c.turn(270)
				c.fwd_straightly(max_secs=1, max_steps=20)
			finally:
				update_img(camera)
