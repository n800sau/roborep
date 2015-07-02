#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

import json
import time
import redis
import traceback
import picamera

COMMAND_QUEUE = 'camera'
UPDATE_IMG_PERIOD = 1
REDUCIBLE = ['image']


class picamserver:

	def __init__(self):
		self.debug = True
		self.r = redis.Redis()
		self.camera_busy = False
		self.camera = picamera.PiCamera()
		self.camera.resolution = (80, 60)
		self.use_video_port = True

	def check_rcommands(self):
		rs = {}
		commands = self.r.lrange(COMMAND_QUEUE, 0, -1)
		if commands:
			self.r.ltrim(COMMAND_QUEUE, 0, len(commands))
			# reduce commands
			for cmd in commands:
				if cmd['command'] in REDUCIBLE:
					rs[cmd['command']] = cmd
				elif cmd['command'] == 'adjust':
					for k,v in cmd['params']:
						if k == 'brightness':
							self.camera.brightness = int(v)
						elif k == 'contrast':
							self.camera.contrast = int(v)
						elif k == 'shutter':
							v = int(v)
							if v > 0:
								self.camera.framerate = Fraction(1, v)
								self.camera.shutter_speed = v * 1000000
								self.camera.exposure_mode = 'off'
								self.camera.iso = 800
								self.use_video_port = False
							else:
								self.camera.exposure_mode = 'auto'
								self.use_video_port = True
		return rs.values()

	def img_fname(self, ndx):
		return os.path.expanduser('~/public_html/picam_%d.jpg' % ndx)

	def execute_cmd(self, cmd):
		self.dbprint('execute %s' % cmd)
		command = cmd['command']
		params = cmd.get('params', {})
		proc = getattr(self, 'pub_' + command, None)
		if proc:
			proc(**params)

	def update_img(self):
		self.camera_busy = True
		try:
			self.camera.capture(self.img_fname(0), use_video_port=True)
		finally:
			self.camera_busy = False

	def run(self):
		t = time.time()
		while True:
			try:
				rcmd = self.check_rcommands()
				if rcmd:
#					print 'chan=', rcmd[0]
#					print 'cmd=', rcmd[1]
					self.translate_cmd(rcmd[1])
				elif not self.camera_busy:
					if time.time() - t > UPDATE_IMG_PERIOD:
						self.update_img()
						t = time.time()
			except Exception, e:
				print e
				time.sleep(2)

if __name__ == '__main__':

	c = picamserver()
	c.run()

