#!/usr/bin/env python

import os, glob, json, time
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
import numpy as np
from matplotlib import pyplot as plt

time_mark = int(time.time())

img_path = os.path.join(os.path.expanduser('~/sshfs/asus/root/sdb1/garage/2015-12-02'), '*.jpg')
out_path = os.path.join(os.path.dirname(__file__), 'output')

def write_image(image, fname, fnames):

	fn = os.path.join('images', fname)
	fnames.append(fn)
	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(image, width=160))


conf = json.load(file('conf.json'))

fnames = []

def process_images(fname1, fname2):

	rs = False

	image1 = cv2.imread(fname1)
	image2 = cv2.imread(fname2)

	gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
	gray1 = cv2.GaussianBlur(gray1, (21, 21), 0)

	gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
	gray2 = cv2.GaussianBlur(gray2, (21, 21), 0)

	avg = gray1.copy().astype("float")

	# accumulate the weighted average between the current frame and
	# previous frames, then compute the difference between the current
	# frame and running average
	cv2.accumulateWeighted(gray2, avg, 0.5)
	frameDelta = cv2.absdiff(gray2, cv2.convertScaleAbs(avg))

	# threshold the delta image, dilate the thresholded image to fill
	# in holes, then find contours on thresholded image
	thresh = cv2.threshold(frameDelta, conf["delta_thresh"], 255, cv2.THRESH_BINARY)[1]
	thresh = cv2.dilate(thresh, None, iterations=2)
	(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	good_cnts = []
	# loop over the contours
	for c in cnts:

		area = cv2.contourArea(c)
		# if the contour is too small, ignore it
		if area < conf["min_area"]:
			continue

		# compute the bounding box for the contour, draw it on the frame,
		# and update the text
		(x, y, w, h) = cv2.boundingRect(c)
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.01 * peri, True)

		if w > conf['max_width'] or h > conf['max_height']:
			continue

		good_cnts.append({'c':c, 'x': x, 'y': y, 'w': w, 'h': h, 'area': area})

		cv2.drawContours(image1, [approx], -1, (0, 0, 128), 2)
		cv2.rectangle(image1, (x, y), (x + w, y + h), (0, 255, 0), 2)

		write_image(image1, 'frame_%d.png' % (len(fnames)+1), fnames)

		rs = True

	return rs


img_start = 0
img_count = 100
fname1 = None
n = 5
for fn in glob.glob(img_path)[img_start:img_start+img_count]:

	if not fname1 is None:
		if process_images(fname1, fn):
			n -= 1
			if n < 0:
				break
	fname1 = fn

print '%d files' % len(fnames)

json.dump({
	'files': fnames,
	'title': os.path.splitext(os.path.basename(__file__))[0],
	'time_mark': time_mark,
}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)
