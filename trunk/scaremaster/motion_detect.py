#!/usr/bin/env python

from __future__ import division

import picamera, time, os, sys, cv2, imutils, traceback
import numpy as np
from scipy.misc import imresize
from PIL import Image
from picamera.array import PiRGBArray, PiYUVArray

basepath = os.path.expanduser('~/sshfs/asus/root/rus_hard/scaremaster/motion')

motion_dtype = np.dtype([
	('x', 'i1'),
	('y', 'i1'),
	('sad', 'u2'),
	])

class MyMotionDetector(object):
	def __init__(self, camera):
		self.camera = camera
#		width, height = camera.resolution
#		print camera.resolution
		width, height = 320, 240
		self.cols = (width + 15) // 16
		self.cols += 1 # there's always an extra column
		self.rows = (height + 15) // 16
		self.mask = np.ones((self.rows, self.cols), dtype=np.uint8)
		margin = int(self.cols * 0.2)
		self.mask[:, margin:self.cols - margin] = 0
		self.nchanges = int((self.mask.size - self.mask.sum()) * 0.1)

	def write(self, s):
		try:
			# Load the motion data from the string to a numpy array
			data = np.fromstring(s, dtype=motion_dtype)
			# Re-shape it and calculate the magnitude of each vector
			data = data.reshape((self.rows, self.cols))
			data = np.ma.array(data, mask=self.mask)
			data = np.sqrt(
				np.square(data['x'].astype(np.float)) +
				np.square(data['y'].astype(np.float))
				).clip(0, 255).astype(np.uint8)
			# If there're more than 10 vectors with a magnitude greater
			# than 60, then say we've detected motion
			detected = data > 40
			# 10% changes
			if detected.sum() > self.nchanges:
				print('M!')
				with PiYUVArray(self.camera, size=(self.camera.resolution)) as rawCapture:
					t = time.time()
					self.camera.capture(rawCapture, format="yuv", use_video_port=True)
					print('Got it in %d sec' % (time.time() - t))
					bname = time.strftime('%Y-%m-%d_%H:%M:%S')
					frame = rawCapture.array
					frame = cv2.cvtColor(frame, cv2.COLOR_YUV2RGB)
					cv2.imwrite(os.path.join(basepath, bname + '.jpg'), frame)
					small = imutils.resize(frame, width=320)
					detected_mask = imresize((detected * 255).astype(np.uint8), small.shape, 'nearest')
					detected_small = cv2.bitwise_and(small, small, mask = detected_mask)
					cv2.imwrite(os.path.join(basepath, bname + '_det_small.jpg'), detected_small)
#	#				print('mask size=%s, type=%s, img size=%s, type=%s' % (mask.shape, mask.dtype, small.shape, small.dtype))
#	#				small = cv2.bitwise_and(small, small, mask = mask.astype(np.uint8))
					mask = cv2.resize((self.mask == 0).astype(np.uint8), (small.shape[1], small.shape[0]))
					small = cv2.bitwise_and(small, small, mask = mask)
					cv2.imwrite(os.path.join(basepath, bname + '_small.jpg'), small)
					cv2.imwrite(os.path.join(basepath, bname + '_motion.jpg'), detected_mask)
					rawCapture.truncate(0)
#	#			self.camera.capture(os.path.join(basepath, bname + '.data'), 'yuv')
#	#			self.camera.capture(os.path.join(basepath, bname + '.jpg'), use_video_port=True)
#	#			detected = cv2.cvtColor(frame, cv2.COLOR_BIN2GRAY)
#	#			img = Image.fromarray(detected * 255)
#	#			img.resize((320, 240)).save(os.path.join(basepath, bname + '_motion.jpg'))
			# Pretend we wrote all the bytes of s
		except Exception, e:
			traceback.print_exc()
		return len(s)

for i in xrange(10):
	with picamera.PiCamera() as camera:
#		camera.resolution = (2592, 1944)
		camera.resolution = (640, 480)
		camera.framerate = 15
		camera.zoom = (0.2, 0., 0.6, 1.)
		camera.start_recording(
			# Throw away the video data, but make sure we're using H.264
			'/dev/null', format='h264', resize=(320, 240),
			# Record motion data to our custom output object
			motion_output=MyMotionDetector(camera)
			)
		camera.wait_recording(300)
		camera.stop_recording()
