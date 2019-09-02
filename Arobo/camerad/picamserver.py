#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

import cv2
import datetime
import json
import traceback
import time
import shutil
import redis
import traceback
from fractions import Fraction
import numpy as np
import picamera
from picamera.array import PiRGBArray

COMMAND_QUEUE = 'camera'
UPDATE_IMG_PERIOD = 1200
REDUCIBLE = ['update', 'primeupdate']


class picamserver:

	def __init__(self):
		self.debug = True
		self.r = redis.Redis()
		self.camera = picamera.PiCamera()
		self.camera.resolution = (80, 60)
		self.settings = {}

	def dbprint(self, text, force=False):
		if self.debug or force:
			print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

	def check_rcommands(self):
		rs = {}
		commands = self.r.lrange(COMMAND_QUEUE, 0, -1)
		if commands:
			self.r.ltrim(COMMAND_QUEUE, len(commands), -1)
			# reduce commands
			for cmd in commands:
				try:
					cmd = json.loads(cmd)
				except Exception:
					self.dbprint(traceback.format_exc())
					continue
				if cmd['command'] in REDUCIBLE:
					rs[cmd['command']] = cmd
				elif cmd['command'] == 'adjust':
					for k,v in cmd['params'].items():
						if k == 'brightness':
							self.settings['brightness'] = int(v)
						elif k == 'contrast':
							self.settings['contrast'] = int(v)
						elif k == 'shutter':
							v = int(v)
							if v > 0:
								self.settings['framerate'] = Fraction(1, v)
								self.settings['shutter_speed'] = v * 1000000
								self.settings['exposure_mode'] = 'off'
								self.settings['iso'] = 800
								self.settings['use_video_port'] = False
							else:
								self.settings['iso'] = 0
								self.settings['exposure_mode'] = 'auto'
								self.settings['use_video_port'] = True
					self.dbprint('Adjust: %s' % self.settings)
		return rs.values()

	def apply_settings(self):
		for k,v in self.settings.items():
			if k not in ['use_video_port']:
				setattr(self.camera, k, v)
				del self.settings[k]

	def img_fname(self, ndx):
		return os.path.expanduser('~/public_html/picam_%d.jpg' % ndx)

	def img_fname_tmp(self, ndx):
		return os.path.expanduser('~/public_html/tmp_picam_%d.jpg' % ndx)

	def threshold_image(self, img, fname, sti):
		style = (
			cv2.THRESH_BINARY,
			cv2.THRESH_BINARY_INV,
			cv2.THRESH_TRUNC,
			cv2.THRESH_TOZERO,
			cv2.THRESH_TOZERO_INV,
		)
		bimg = cv2.medianBlur(img, 5)
		ret,bimg = cv2.threshold(bimg, 127, 255, style[sti])
		cv2.imwrite(fname, bimg)
		return fname

	def filter_image_color(self, img, fname, fmask):
		# Convert BGR to HSV
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		#H - 0-360 .. 0-180 (coef = *2)
		#S - 0-100 .. 0-255 (coef = 0.39)
		#V - 0-100 .. 0-255 (coef = 0.39)

		# define range of blue color in HSV
		lower_orange = np.array([6,50,50])
		upper_orange = np.array([16,255,255])

		# Threshold the HSV image to get only orange colors
		mask = cv2.inRange(hsv, lower_orange, upper_orange)

		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(img, img, mask=mask)

		cv2.imwrite(fmask, mask)
		cv2.imwrite(fname, res)

	def photo_image(self, img, fname):
		cv2.imwrite(fname, img)
		return fname

	def fast_image(self, fname):
		self.camera.exposure_mode = 'auto'
		self.camera.capture(fname, use_video_port=True)
		return fname

	def capture_cvimage(self):
		# initialize the camera and grab a reference to the raw camera capture
		stream = PiRGBArray(self.camera)
		# grab an image from the camera
		self.camera.capture(stream, format="bgr", use_video_port=self.settings.get('use_video_port',True))
		return stream.array

	def pub_update(self):
		self.apply_settings()
		img = self.capture_cvimage()
		i = 1
		shutil.move(self.photo_image(img, self.img_fname_tmp(i)), self.img_fname(i))
		i += 1
		shutil.move(self.threshold_image(img, self.img_fname_tmp(i), 1), self.img_fname(i))
		i += 1
		shutil.move(self.threshold_image(img, self.img_fname_tmp(i), 2), self.img_fname(i))
		i += 1
		shutil.move(self.threshold_image(img, self.img_fname_tmp(i), 3), self.img_fname(i))
		i += 1
		shutil.move(self.threshold_image(img, self.img_fname_tmp(i), 4), self.img_fname(i))
		i += 1
		self.filter_image_color(img, self.img_fname_tmp(i), self.img_fname_tmp(i+1))
		shutil.move(self.img_fname_tmp(i), self.img_fname(i))
		i += 1
		shutil.move(self.img_fname_tmp(i), self.img_fname(i))
		i += 1

	def pub_primeupdate(self):
		self.update_img()

	def execute_cmd(self, cmd):
		self.dbprint('execute %s' % cmd)
		command = cmd['command']
		params = cmd.get('params', {})
		proc = getattr(self, 'pub_' + command, None)
		if proc:
			proc(**params)

	def update_img(self):
		self.apply_settings()
		shutil.move(self.fast_image(self.img_fname_tmp(0)), self.img_fname(0))
		self.dbprint('image 0 updated at %s' % time.strftime('%c'))

	def run(self):
		t = time.time()
		while True:
			try:
				cmdlist = self.check_rcommands()
				if cmdlist:
					for cmd in cmdlist:
						self.execute_cmd(cmd)
				elif time.time() - t > UPDATE_IMG_PERIOD:
					self.update_img()
					t = time.time()
				time.sleep(0.2)
			except Exception:
				self.dbprint(traceback.format_exc())
				time.sleep(2)

if __name__ == '__main__':

	c = picamserver()
	c.run()

