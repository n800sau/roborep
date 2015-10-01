#!/usr/bin/env python

import sys, os, time, redis

#import picamera
from lib.camera import update_img
from lib.utils import dbprint
from lib.marker import collect_markers, use_camera, release_camera, make_shot

if __name__ == '__main__':

	r = redis.Redis()
	use_camera(r)
	time.sleep(4)
	try:
		markers = collect_markers(r, fpath = os.path.join(os.path.expanduser('~/public_html'), 'pic0.jpg'))
	finally:
		time.sleep(1)
		make_shot(r)
		release_camera(r)
#		update_img(picamera.PiCamera())
