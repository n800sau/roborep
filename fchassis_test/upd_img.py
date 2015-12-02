#!/usr/bin/env python

import sys, os, time, redis

import picamera
from lib.camera import update_img
from lib.utils import dbprint
from lib.marker import collect_markers, use_camera, release_camera, make_shot
from lib.utils import html_data_path

if __name__ == '__main__':

	if 1:
		r = redis.Redis()
		use_camera(r)
#		use_camera(r, brightness=90, contrast=90)
		time.sleep(4)
		try:
			markers = collect_markers(r, fpath = html_data_path('markers.jpg'))
		finally:
			time.sleep(1)
			make_shot(r)
			release_camera(r)
	else:
		cam = picamera.PiCamera()
		time.sleep(2)
		update_img(cam)
