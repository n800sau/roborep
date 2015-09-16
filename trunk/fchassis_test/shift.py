#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img, FeatureProcess, capture_cvimage

from lib.frobo import frobo

if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	right = int(sys.argv[1])

	with frobo(s_port) as c:
		#c.debug = False

		with picamera.PiCamera() as camera:

				fp = FeatureProcess(camera)
				try:
					update_img(camera, 'pic0.jpg')
					h = c.compass.heading()
					c.turn_in_steps((h + (1 if right else -1) * 90) % 360)
					c.fwd_straight(max_secs=1, max_steps=c.m2steps(0.4))
					c.turn_in_steps(h)
					update_img(camera, 'pic1.jpg')
					json.dump(c.dots, file('dots.json', 'w'), indent=2)
				finally:
					update_img(camera)
