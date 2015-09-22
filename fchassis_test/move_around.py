#!/usr/bin/env python

import sys, os, time, json, redis, random

import picamera
from lib.hmc5883l import hmc5883l
from lib.utils import dbprint
from lib.camera import update_img
from lib.marker import use_camera, release_camera, collect_markers

from lib.frobo import frobo

if __name__ == '__main__':

	random.seed()

	clockwise = int(sys.argv[1])

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with frobo(s_port) as c:
		#c.debug = False

		r = redis.Redis()
		use_camera(r)
		time.sleep(4)
		try:
			dbprint('BEFORE %s cm to %s' % (c.curr_dist, c.compass.heading()))
			for i in range(5):
				c.fwd_straight(max_secs=20, max_steps=c.m2steps(random.randint(10, 100)), power=100)
				c.find_distance(60, clockwise=random.choice((True, False)))
				markers = collect_markers(r, fpath = os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % i))
				if markers:
					dbprint('Step %d. Found %d markers' % (i, len(markers)))
			dbprint('AFTER %s cm to %s' % (c.curr_dist, c.compass.heading()))
			json.dump(c.dots, file('dots.json', 'w'), indent=2)
		finally:
			release_camera(r)
			update_img(picamera.PiCamera())
