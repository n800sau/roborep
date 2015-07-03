#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

import datetime
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
				except Exception, e:
					self.dbprint(e)
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
								self.settings['exposure_mode'] = 'auto'
								self.settings['use_video_port'] = True
					self.dbprint('Adjust: %s' % self.settings)
		return rs.values()

	def pub_image(self, ndx=1):
		self.camera_busy = True
		try:
			for k,v in self.settings.items():
				if k not in ['use_video_port']:
					setattr(self.camera, k, v)
					del self.settings[k]
			self.camera.capture(self.img_fname(ndx), use_video_port=settings.get('use_video_port',True))
		finally:
			self.camera_busy = False

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
			self.dbprint('image 0 updated at %s' % time.strftime('%c'))
		finally:
			self.camera_busy = False

	def run(self):
		t = time.time()
		while True:
			try:
				cmdlist = self.check_rcommands()
				if cmdlist:
					for cmd in cmdlist:
						self.translate_cmd(cmd)
				elif not self.camera_busy:
					if time.time() - t > UPDATE_IMG_PERIOD:
						self.update_img()
						t = time.time()
			except Exception, e:
				self.dbprint(e)
				time.sleep(2)

if __name__ == '__main__':

	c = picamserver()
	c.run()

