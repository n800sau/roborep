#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

import json
import time
import datetime
import math
import signal
import redis
import struct
import traceback
import threading
import Queue
from PyMata.pymata import PyMata

import numpy as np
import picamera
import picamera.array
from PIL import Image
import cv2
from hmc5883l import hmc5883l

import logging
logging.basicConfig(filename='fchassis_bmp.log',level=logging.DEBUG)


COUNT_PER_REV = 20.0
WHEEL_DIAMETER = 0.065
BASELINE = 0.14

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

ENC_STEP = WHEEL_DIAMETER * math.pi / COUNT_PER_REV


REDIS_CHANNEL = 'command'
SENSORS_SHOW_PERIOD = 1

MIN_PWR = 70
MAX_PWR = 255

LEFT_MOTOR_1 = 6
LEFT_MOTOR_2 = 5

RIGHT_MOTOR_1 = 10
RIGHT_MOTOR_2 = 9

# encoder pins
ENCODER_L = 2
ENCODER_R = 3

# Indices into callback return data list
DEVICE = 0
PIN = 1
DATA = 2

class fchassis:

	def __init__(self, s_dev):

		self.debug = True
		self.left_dir = None
		self.right_dir = None


		self.enc_data = {
			ENCODER_L: {
				'name': 'left',
				'lvl': 0,
				'dir': 0,
				'tick_time': time.time(),
				'count': 0,
				'pwr': 0,
			},
			ENCODER_R: {
				'name': 'right',
				'lvl': 0,
				'dir': 0,
				'tick_time': time.time(),
				'count': 0,
				'pwr': 0,
			},
		}

		# Create a PyMata instance
		#self.board = PyMata(s_dev)
		self.board = PyMata(s_dev, verbose=False)
		self.r = redis.Redis()

		signal.signal(signal.SIGINT, self.signal_handler)

		# configure firmata for i2c on an UNO
		self.board.i2c_config(0, self.board.ANALOG, 4, 5)

		self.board.set_pin_mode(ENCODER_L, self.board.INPUT, self.board.DIGITAL)
		# Arm the digital latch to detect when the button is pressed
		self.arm_latch(ENCODER_L, True)

		self.board.set_pin_mode(ENCODER_R, self.board.INPUT, self.board.DIGITAL)
		# Arm the digital latch to detect when the button is pressed
		self.arm_latch(ENCODER_R, True)

	def __enter__(self):
		return self

	def __exit__(self, type, value, traceback):
		# close the interface down cleanly
		print 'Exiting...'
		try:
			self.board.close()
		except:
			pass
		time.sleep(1)

	def signal_handler(self, sig, frame):
		print('You pressed Ctrl+C!!!!')
		if self.board is not None:
			self.board.reset()
		sys.exit(0)

	def dbprint(self, text, force=False):
		if self.debug or force:
			print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

	def measure(self):
		compass = self.get_last('hmc5883l', 2)
		if compass:
			self.last_heading = compass['heading_degrees']
		return self.last_heading

	def reset_counters(self):
		self.enc_data[ENCODER_L]['count'] = 0
		self.enc_data[ENCODER_R]['count'] = 0

	def steps_counted(self):
		return max(abs(self.enc_data[ENCODER_L]['count']), abs(self.enc_data[ENCODER_R]['count']))

	def adjust_pwr(self, pwr):
		self.dbprint('output=%f' % self.get())

	def lpwr(self, pwr):
		return int(min(100, pwr) / 100. * (MAX_PWR - MIN_PWR) + MIN_PWR)

	def right_move(self, direct, pwr):
		if pwr == 0:
				self.board.set_pin_mode(RIGHT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
				self.board.set_pin_mode(RIGHT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
				self.board.digital_write(RIGHT_MOTOR_1, 0)
				self.board.digital_write(RIGHT_MOTOR_2, 0)
#				self.dbprint('right PWM stopped')
		else:
			lpwr = self.lpwr(pwr)
			if direct:
				self.board.set_pin_mode(RIGHT_MOTOR_1, self.board.PWM, self.board.DIGITAL)
				self.board.set_pin_mode(RIGHT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
				self.board.analog_write(RIGHT_MOTOR_1, lpwr)
				self.board.digital_write(RIGHT_MOTOR_2, 0)
			else:
				self.board.set_pin_mode(RIGHT_MOTOR_2, self.board.PWM, self.board.DIGITAL)
				self.board.set_pin_mode(RIGHT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
				self.board.analog_write(RIGHT_MOTOR_2, lpwr)
				self.board.digital_write(RIGHT_MOTOR_1, 0)
		self.enc_data[ENCODER_R]['pwr'] = pwr
		if pwr > 0:
			self.right_dir = direct
			self.enc_data[ENCODER_R]['dir'] = direct

	def left_move(self, direct, pwr):
		if pwr == 0:
				self.board.set_pin_mode(LEFT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
				self.board.set_pin_mode(LEFT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
				self.board.digital_write(LEFT_MOTOR_1, 0)
				self.board.digital_write(LEFT_MOTOR_2, 0)
#				self.dbprint('left PWM stopped')
		else:
			lpwr = self.lpwr(pwr)
			if direct:
				self.board.set_pin_mode(LEFT_MOTOR_1, self.board.PWM, self.board.DIGITAL)
				self.board.set_pin_mode(LEFT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
				self.board.analog_write(LEFT_MOTOR_1, lpwr)
				self.board.digital_write(LEFT_MOTOR_2, 0)
			else:
				self.board.set_pin_mode(LEFT_MOTOR_2, self.board.PWM, self.board.DIGITAL)
				self.board.set_pin_mode(LEFT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
				self.board.analog_write(LEFT_MOTOR_2, lpwr)
				self.board.digital_write(LEFT_MOTOR_1, 0)
		self.enc_data[ENCODER_L]['pwr'] = pwr
		if pwr > 0:
			self.left_dir = direct
			self.enc_data[ENCODER_L]['dir'] = direct

	def arm_latch(self, pin, high):
		self.board.set_digital_latch(pin, self.board.DIGITAL_LATCH_HIGH if high else self.board.DIGITAL_LATCH_LOW, lambda data, pin=pin: self.cb_encoder(data, pin))

	def cb_encoder(self, data, pin):
		if data[2] != self.enc_data[pin]['lvl'] and data[2]:
			t = time.time()
			dt = t-self.enc_data[pin]['tick_time']
			step = (1 if self.enc_data[pin]['dir'] else -1)
			self.enc_data[pin]['count'] += step
#			self.dbprint("pin: %d (%s), dt:%s, v:%.2f, cnt:%d" % (pin, self.enc_data[pin]['name'], dt, step * ENC_STEP / dt, self.enc_data[pin]['count']))
			self.enc_data[pin]['tick_time'] = t
		self.enc_data[pin]['lvl'] = data[2]
		self.arm_latch(pin, not data[2])

	def is_stopped(self):
		return self.enc_data[ENCODER_L]['pwr'] == 0 and self.enc_data[ENCODER_R]['pwr'] == 0

	def stop(self):
		self.keep_heading = None
		if not self.is_stopped():
			self.left_move(0, 0)
			self.right_move(0, 0)
			self.dbprint('both stopped')

	def get_last(self, sname, no_older_than=None):
		rs = self.r.lrange('%s.js.obj' % sname, 0, 0)
		if rs:
			rs = json.loads(rs[0])
			if not no_older_than is None:
#			print float(rs['timestamp']) + no_older_than, time.time()
				if float(rs['timestamp']) + no_older_than < time.time():
					rs = None
		return rs

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
	moutput = DetectMotion(camera)
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
		c.dbprint('finish')
		c.stop()
		camera.stop_recording(splitter_port=1)
#		camera.stop_recording(splitter_port=2)

def turn(c):
	c.left_move(False, 50)
	c.right_move(True, 50)
	time.sleep(1)
	c.stop()

def update_img(camera):
	camera.exposure_mode = 'off'
	camera.brightness = 50
	camera.contrast = 50
	camera.capture(os.path.expanduser('~/public_html/picam_0.jpg'), use_video_port=True)


if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with fchassis(s_port) as c:

		with picamera.PiCamera() as camera:

			try:
				experiment1(c, camera)
#				turn(c)
			finally:
				update_img(camera)
