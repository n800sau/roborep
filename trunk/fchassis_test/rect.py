#!/usr/bin/env python

import sys, os, time, json

import picamera
from lib.utils import dbprint
from lib.camera import update_img, FeatureProcess, capture_cvimage

from lib.frobo import frobo

if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		#c.debug = False

		with picamera.PiCamera() as camera:

			fp = FeatureProcess(camera)
			try:
				offs = c.pwr_offsets()
				update_img(camera, 'pic0.jpg')
				c.turn(350, power_offsets=offs)
				time.sleep(1)
				update_img(camera, 'pic1.jpg')
				fp.percent()
				c.fwd_straight(max_secs=1, max_steps=50, power_offsets=offs)
				update_img(camera, 'pic2.jpg')
				time.sleep(1)
				c.turn(90, power_offsets=offs)
				time.sleep(1)
				c.fwd_straight(max_secs=1, max_steps=50, power_offsets=offs)
				update_img(camera, 'pic3.jpg')
				time.sleep(1)
				c.turn(210, power_offsets=offs)
				time.sleep(1)
				c.fwd_straight(max_secs=1, max_steps=50, power_offsets=offs)
				update_img(camera, 'pic4.jpg')
				time.sleep(1)
				c.turn(270, power_offsets=offs)
				time.sleep(1)
				c.fwd_straight(max_secs=1, max_steps=50, power_offsets=offs)
				update_img(camera, 'pic5.jpg')
				dbprint('Matches %s%%' % fp.percent())
				json.dump(c.dots, file('dots.json', 'w'), indent=2)
			finally:
				update_img(camera)
