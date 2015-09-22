#!/usr/bin/env python

import sys, os, time

import picamera
from lib.utils import dbprint
from lib.camera import update_img

from lib.frobo import frobo


if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
#		c.debug = False

		with picamera.PiCamera() as camera:

			try:
				c.tick_right()
			finally:
				update_img(camera)
