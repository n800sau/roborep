#!/usr/bin/env python

import os, sys
import cv2
import numpy as np
from imutils import paths, resize
from sklearn.neighbors import KNeighborsClassifier
from sklearn.cluster import KMeans

INPUT = 'input'
OUTPUT = 'output'

AVGFNAME = 'avg.npy'
avg = None
if os.path.exists(AVGFNAME):
	avg = np.load(AVGFNAME)

for ifname in paths.list_images(INPUT):
	print ifname
	image = cv2.imread(ifname)
#	resized = resize(image, width=360)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	if avg is None:
		print("[INFO] starting background model...")
		avg = gray.copy().astype("float")
		continue

	# accumulate the weighted average between the current frame and
	# previous frames, then compute the difference between the current
	# frame and running average
	cv2.accumulateWeighted(gray, avg, 0.5)
	frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avg))

	# threshold the delta image, dilate the thresholded image to fill
	# in holes, then find contours on thresholded image
	thresh = cv2.threshold(frameDelta,40, 255, cv2.THRESH_BINARY)[1]

#	thresh = cv2.adaptiveThreshold(frameDelta, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
	thresh = cv2.dilate(thresh, None, iterations=2)
	(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	oimage = resize(image, width=320)

	n = 0
	# loop over the contours
	for c in sorted(cnts, key=lambda v: cv2.contourArea(v), reverse=True)[:1]:

		area = cv2.contourArea(c)

		# if the contour is too small or too big, ignore it
		if area < 40000 or area > 1000000:
			continue

		# compute the bounding box for the contour, draw it on the frame,
		# and update the text
		(x, y, w, h) = cv2.boundingRect(c)

		if w < 200 or h < 100:
			continue

		xcoef = oimage.shape[1] / float(image.shape[1])
		ycoef = oimage.shape[0] / float(image.shape[0])

		x = int(x * xcoef)
		w = int(w * xcoef)
		y = int(y * ycoef)
		h = int(h * ycoef)

#		print x,y,w,h

		n += 1
		cv2.rectangle(oimage, (x, y), (x + w, y + h), (0, 0, 255), 2)

		print 'area', area

	if n:
		cv2.imwrite(os.path.join(OUTPUT, 'motion_' + os.path.basename(ifname)), oimage)
