#!/usr/bin/env python

import os, sys
import cv2
import numpy as np
from imutils import paths, resize

INPUT = 'input'
OUTPUT = 'output'

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)

	# apply automatic Canny edge detection using the computed median
	slower = int(max(0, (1.0 - sigma) * v))
	supper = int(min(255, (1.0 + sigma) * v))
#	print('lower=%s, upper=%s' % (slower, supper))
	edged = cv2.Canny(image, slower, supper)

	# return the edged image
	return edged

for ifname in paths.list_images(INPUT):
	print ifname
	image = cv2.imread(ifname)
#	resized = resize(image, width=360)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#	gray = cv2.blur(gray, (5, 5))
	blurred = cv2.GaussianBlur(gray, (7, 7), 0)
	(T, threshInv) = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
	oimage = threshInv
#	blurred = cv2.bitwise_and(blurred, blurred, mask=threshInv)
#	gX = cv2.Sobel(gray, ddepth=cv2.CV_64F, dx=1, dy=0)
#	gY = cv2.Sobel(gray, ddepth=cv2.CV_64F, dx=0, dy=1)
#	gX = cv2.convertScaleAbs(gX)
#	gY = cv2.convertScaleAbs(gY)
#	oimage = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)
#	oimage = auto_canny(oimage)
	(cnts, _) = cv2.findContours(oimage.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
	for c in cnts:
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.01 * peri, True)
		area = cv2.contourArea(c)
		if len(approx) == 4 and area > 6:
			print 'approx size=', len(approx), ",", area
			cv2.drawContours(image, [c], -1, (0, 255, 255), 5)
			(x, y, w, h) = cv2.boundingRect(approx)
			print 'Found', x, y, w, h
#		cv2.putText(resized, "Rectangle", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
			cv2.imwrite(os.path.join(OUTPUT, os.path.basename(ifname)), resize(image, width=320))
