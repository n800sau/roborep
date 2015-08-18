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
				update_img(camera, 'pic0.jpg')
				c.fwd_straightly(max_secs=3, max_steps=100)
				update_img(camera, 'pic1.jpg')
				c.turn(90)
				c.fwd_straightly(max_secs=3, max_steps=100)
				update_img(camera, 'pic2.jpg')
				c.turn(180)
				c.fwd_straightly(max_secs=3, max_steps=100)
				update_img(camera, 'pic3.jpg')
				c.turn(270)
				c.fwd_straightly(max_secs=3, max_steps=100)
				update_img(camera, 'pic4.jpg')
			finally:
				update_img(camera)
