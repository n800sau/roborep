#!/usr/bin/env python

import os, glob, json, time, random
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
import numpy as np
from matplotlib import pyplot as plt

time_mark = int(time.time())

img_path = os.path.join(os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-01-23'), '*.jpg')
out_path = os.path.join(os.path.dirname(__file__), 'output')

random.seed()

colors = [(random.randint(0, 255), random.randint(0, 255), random.randint(100, 255)) for i in range(20)]

def sort_contours(cnts, method="left-to-right"):
	# initialize the reverse flag and sort index
	reverse = False
	i = 0

	# handle if we need to sort in reverse
	if method == "right-to-left" or method == "bottom-to-top":
		reverse = True

	# handle if we are sorting against the y-coordinate rather than
	# the x-coordinate of the bounding box
	if method == "top-to-bottom" or method == "bottom-to-top":
		i = 1

	# construct the list of bounding boxes and sort them from top to
	# bottom
	boundingBoxes = [cv2.boundingRect(c) for c in cnts]
	(cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),
		key=lambda b:b[1][i], reverse=reverse))

	# return the list of sorted contours and bounding boxes
	return (cnts, boundingBoxes)

def draw_contour(image, c, i):
	# compute the center of the contour area and draw a circle
	# representing the center
	M = cv2.moments(c)
	if M['m00'] != 0:
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])

		cv2.drawContours(image, [c], -1, colors[i], 2)
		# draw the countour number on the image
		cv2.putText(image, "#{}".format(i + 1), (cX - 20, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, colors[i], 2)

	# return the image with the contour number drawn on it
	return image

def write_image(image, fname, fnames):

	fn = os.path.join('images', fname)
	fnames.append(fn)
	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(image, width=160))


def process_image(fname):

	bname = os.path.basename(fname)

	image = cv2.imread(fname)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# load the image and initialize the accumulated edge image
	image = cv2.imread(fname)
	accumEdged = np.zeros(image.shape[:2], dtype="uint8")

	# loop over the blue, green, and red channels, respectively
	for chan in cv2.split(image):
		# blur the channel, extract edges from it, and accumulate the set
		# of edges for the image
		chan = cv2.medianBlur(chan, 7)
#		chan = cv2.medianBlur(chan, 11)
		edged = cv2.Canny(chan, 50, 150)
		accumEdged = cv2.bitwise_or(accumEdged, edged)

	write_image(accumEdged, bname + '_edged.png', fnames)

	# find contours in the accumulated image, keeping only the largest ones
	(cnts, _) = cv2.findContours(accumEdged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]

	orig = image.copy()

	# loop over the (unsorted) contours and draw them
	for (i, c) in enumerate(cnts):
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.06 * peri, True)
		print 'nonsort n points:', len(approx)
#		if len(approx) == 3:
		orig = draw_contour(orig, approx, i)

	write_image(orig, bname + '_orig.png', fnames)

	# sort the contours according to the provided method
	(cnts, boundingBoxes) = sort_contours(cnts)

	# loop over the (now sorted) contours and draw them
	for (i, c) in enumerate(cnts):
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.01 * peri, True)
		print 'sorted n points:', len(approx)
#		if len(approx) == 3:
		draw_contour(image, approx, i)

	write_image(image, bname + '_image.png', fnames)



fnames = []

flist = glob.glob(img_path)
flist = random.sample(flist, min(len(flist), 10))
for fn in flist[:3]:

	process_image(fn)

print '%d files' % len(fnames)

json.dump({
	'files': fnames,
	'title': os.path.splitext(os.path.basename(__file__))[0],
	'time_mark': time_mark,
}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)
