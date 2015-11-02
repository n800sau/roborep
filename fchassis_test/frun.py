#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

import json
import time
import datetime
import math
import traceback
import glob

import numpy as np
import picamera
import picamera.array
from PIL import Image
import cv2
from lib.hmc5883l import hmc5883l

from lib.fchassis import fchassis
from lib.utils import html_data_path

# "FAST/0" - FastFeatureDetector
# "STAR" - StarFeatureDetector
# "SIFT"* - SIFT (nonfree module)
# "SURF" - SURF (nonfree module)
# "ORB"+ - ORB
# "BRISK" - BRISK
# "MSER" - MSER
# "GFTT"+ - GoodFeaturesToTrackDetector
# "HARRIS" - GoodFeaturesToTrackDetector with Harris detector enabled
# "Dense" - DenseFeatureDetector
# "SimpleBlob" - SimpleBlobDetector
DETECTOR = 'GFTT'

# SIFT/0, SURF*, BRIEF+, BRISK*, ORB+, FREAK*
EXTRACTOR = 'BRIEF'


MATCHER = 'BruteForce-Hamming'

REDIS_CHANNEL = 'command'
SENSORS_SHOW_PERIOD = 1

HTML_PATH = os.path.expanduser('~/public_html')

class ImageProcess(picamera.array.PiRGBAnalysis):

	def __init__(self, *args, **kwds):
		self.chassis = kwds.pop('chassis', None)
		super(ImageProcess, self).__init__(*args, **kwds)
		self.img = None
		self.skipped = 0
		self.plist = []
		self.cv_det = cv2.FeatureDetector_create(DETECTOR)
		self.cv_desc = cv2.DescriptorExtractor_create(EXTRACTOR)
		self.matcher = cv2.DescriptorMatcher_create(MATCHER)
		self.compass = hmc5883l(gauss = 4.7, declination = (12, 34))
#		self.cv = cv2.SURF(400)
#		self.matcher = cv2.BFMatcher(cv2.NORM_L2)

	def dbprint(self, text, force=False):
		print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

	def filterMatches(self, kp, matches, ratio = 0.75):
		mkp1, mkp2 = [], []
		for m in matches:
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				m = m[0]
				mkp1.append(self.kp[m.queryIdx])
				mkp2.append(kp[m.trainIdx])
		return zip( mkp1, mkp2 )

	def analyse(self, frame):
		if self.skipped < 5:
			self.skipped += 1
			return
#		frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
#		compass = c.get_last('hmc5883l', 10)
		if self.img is None:
			self.img = frame
			self.kp = self.cv_det.detect(self.img)
			self.kp, self.desc = self.cv_desc.compute(self.img, self.kp)
#			self.kp, self.desc = self.cv.detectAndCompute(self.img, None)
			self.kpl = len(self.kp)
			self.dbprint('init: %s' % (self.compass.degrees(self.compass.heading()),))
#			cv2.imwrite('init.jpg', self.img)
		else:
			kp = self.cv_det.detect(frame)
			kp, desc = self.cv_desc.compute(frame, kp)

#			kp, desc = self.cv.detectAndCompute(frame, None)
			matches = self.matcher.knnMatch(self.desc, trainDescriptors=desc, k=2)
			pairs = self.filterMatches(kp, matches)
			lp = len(pairs)
			r = (lp * 100) / self.kpl
			self.plist.append(r)
			self.dbprint('%s%% - %s' % (r, self.compass.degrees(self.compass.heading())))
#			self.dbprint('%s%% - %s (%s)' % (r, int(compass['rawX']), time.strftime('%H:%M:%S', time.localtime(compass['timestamp']))))
#			self.dbprint('%s%% - %s (%s)' % (r, int(compass['heading_degrees']), time.strftime('%H:%M:%S', time.localtime(compass['timestamp']))))
#			self.dbprint('MX %s' % (np.diff(self.plist)))
#			self.dbprint('MX %s' % ((np.diff(np.sign(np.diff(self.plist))) < 0).nonzero()[0] + 1))


class DetectMotion(picamera.array.PiMotionAnalysis):

	def dbprint(self, text, force=False):
		print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

	def analyse(self, frame):
		xmove = np.mean(frame['x'])
		ymove = np.mean(frame['y'])
		maxdiff = np.max(frame['sad'])
#		if abs(xmove) < 10:
#		self.dbprint('MV %.1f %.1f %.1f' % (xmove, ymove, maxdiff))

def experiment1(c, camera):
	output = ImageProcess(camera, chassis=c)
#	moutput = DetectMotion(camera)
	camera.resolution = (160, 120)
	camera.framerate = 5
	camera.start_recording(output, format='bgr', splitter_port=1)
#	camera.start_recording('/dev/null', format='h264', splitter_port=2, motion_output=moutput)
	try:
#		for i in range(30):
		c.right_move(False, 30)
		c.left_move(True, 30)
#			time.sleep(0.2)
#			c.stop()
#			time.sleep(1)
#		camera.wait_recording(10)
		time.sleep(4)
	finally:
		c.stop()
		dbprint('stopped')
		camera.stop_recording(splitter_port=1)
#		camera.stop_recording(splitter_port=2)

def experiment2(c, camera):
	def filterMatches(base_kp, kp, matches, ratio = 0.75):
		mkp1, mkp2 = [], []
		for m in matches:
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				m = m[0]
				mkp1.append(base_kp[m.queryIdx])
				mkp2.append(kp[m.trainIdx])
		return zip(mkp1, mkp2)

	try:
		clear_img()
		camera.resolution = (160, 120)
		camera.framerate = 5
		cv_det = cv2.FeatureDetector_create(DETECTOR)
		cv_desc = cv2.DescriptorExtractor_create(EXTRACTOR)
		matcher = cv2.DescriptorMatcher_create(MATCHER)
		was_below = False

		compass = hmc5883l(gauss = 4.7, declination = (12, 34))
		old_heading = init_heading = compass.heading()
		hbase_diff = 0
		h_dir = 1

		with picamera.array.PiRGBArray(camera) as stream:
			camera.capture(stream, format='bgr', use_video_port=True)
			base_frame = stream.array
			cv2.imwrite(html_data_path('start.jpg'), base_frame)
			circle_count = 0
			found_count = 0
			base_kp = cv_det.detect(base_frame)
			base_kp, base_desc = cv_desc.compute(base_frame, base_kp)
			base_kpl = len(base_kp)
			for i in range(200):
				c.right_move(True, 40)
				c.left_move(False, 40)
				time.sleep(0.1)
				c.stop()
				time.sleep(0.2)
				with picamera.array.PiRGBArray(camera) as stream:
					camera.capture(stream, format='bgr', use_video_port=True)
					frame = stream.array
				kp = cv_det.detect(frame)
				kp, desc = cv_desc.compute(frame, kp)
				matches = matcher.knnMatch(base_desc, trainDescriptors=desc, k=2)
				pairs = filterMatches(base_kp, kp, matches)
				lp = len(pairs)
				rperc = (lp * 100) / base_kpl

				heading = compass.heading()
				hdiff = abs(heading - init_heading)
				if h_dir > 0:
					if hbase_diff + 5 <= hdiff:
						hbase_diff = hdiff
					elif hbase_diff >= hdiff + 5:
						h_dir = -1
				else:
					if hbase_diff >= hdiff + 5:
						hbase_diff = hdiff
					elif hbase_diff + 5 <= hdiff:
						# found
						dbprint('Circle!!!')
						circle_count += 1
						cv2.imwrite(html_data_path('circle%03d.jpg' % circle_count), frame)
						h_dir = 1

				dbprint('%.2f%% - %.2f (%.2f, dh: %.2f)' % (rperc, heading, hdiff, abs(old_heading - heading) ))
				if was_below:
					if rperc > 60:
						dbprint('Found close')
						was_below = False
						found_count += 1
						cv2.imwrite(html_data_path('found%03d.jpg' % found_count), frame)
				else:
					if rperc < 10:
						was_below = True
				old_heading = heading
	finally:
		c.stop()
		dbprint('stopped')

def turn(c):
	c.left_move(False, 50)
	c.right_move(True, 50)
	time.sleep(1)
	c.stop()

def update_img(camera):
	camera.exposure_mode = 'off'
	camera.brightness = 50
	camera.contrast = 50
	camera.capture(html_data_path('picam_0.jpg'), use_video_port=True)

def unlink(fname):
	try:
		os.unlink(fname)
	except OSError:
		pass

def clear_img():
	unlink(html_data_path('start.jpg'))
	for f in glob.glob(html_data_path('found*.jpg')):
		unlink(f)
	for f in glob.glob(html_data_path('circle*.jpg')):
		unlink(f)

def dbprint(text, force=False):
	print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)


if __name__ == '__main__':

#	import logging
#	logging.basicConfig(filename='fchassis_bmp.log',level=logging.DEBUG)

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with fchassis(s_port) as c:
		c.debug = False

		with picamera.PiCamera() as camera:

			try:
				experiment2(c, camera)
#				turn(c)
			finally:
				update_img(camera)
