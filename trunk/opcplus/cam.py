#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import cv2, random, json, time
from imutils import paths, resize
import redis
import argparse
import numpy as np

OUTPUT_PATH = 'output'

REDIS_KEY = 'LCD_timed_lines'
r = redis.Redis()

#fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
fps = 10
min_area = 300
max_area = 50000
delta_thresh = 5

camera = cv2.VideoCapture(1)
camera.set(cv2.cv.CV_CAP_PROP_FPS, fps)
camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)

print 'Start processing'

t = time.time()

try:

	avg = None
	moveCounter = 0
	i = 0
	while True:

		(grabbed, frame) = camera.read()

		if not grabbed:
			continue

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)

		if i < 1000:
			if avg is None:
				avg = gray.copy().astype("float")
				continue

			cv2.accumulateWeighted(gray, avg, 0.5)
			i += 1

		else:

			frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
			thresh = cv2.threshold(frameDelta, delta_thresh, 255, cv2.THRESH_BINARY)[1]
			thresh = cv2.dilate(thresh, None, iterations=2)
			(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

			found = False
			# loop over the contours
			for c in cnts:

				area = cv2.contourArea(c)
				# if the contour is too small, ignore it
				if area < min_area:
					continue
				if area > max_area:
					continue
				print('Area: %s, Count: %d' % (area, len(c)))

				found = True
				peri = cv2.arcLength(c, True)
				approx = cv2.approxPolyDP(c, 0.01 * peri, True)
				print('contour len: %d' % len(approx))
	#			if len(approx) == 4:
				cv2.drawContours(frame, [approx], -1, (0, 0, 255), 2)

			if found:
				moveCounter += 1
				print 'Found', moveCounter
				if moveCounter > 5:
					cv2.imwrite(os.path.join(OUTPUT_PATH, time.strftime('found%d_%H:%M:%S.jpg')), frame)
			else:
				moveCounter = 0
			cv2.accumulateWeighted(gray, avg, 0.5)

finally:
	print 'Finished (%d frame/sec)' % (i / (time.time() - t))
	camera.release()
