#!/usr/bin/env python

import os, glob, json
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
import numpy as np

img_path = os.path.join(os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-03-25'), '*.jpg')
out_path = os.path.join(os.path.dirname(__file__), 'output')

def process_image(fname):

	image = cv2.imread(fname)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	fnames = []

	# extract the Value component from the HSV color space and apply adaptive thresholding
	# to reveal the characters on the license plate
	V = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))[2]
	thresh = threshold_adaptive(V, 30, offset=15).astype("uint8") * 255
	thresh = cv2.bitwise_not(thresh)

	labels = measure.label(thresh, neighbors=8, background=0)
	mask = np.zeros(thresh.shape, dtype="uint8")

	# loop over the unique components
	for (i, label) in enumerate(np.unique(labels)):
		# if this is the background label, ignore it
		if label == -1:
			print("[INFO] label: -1 (background)")
			continue

		# otherwise, construct the label mask to display only connected components for
		# the current label
#		print("[INFO] label: {} (foreground)".format(i))
		labelMask = np.zeros(thresh.shape, dtype="uint8")
		labelMask[labels == label] = 255
		numPixels = cv2.countNonZero(labelMask)

		# if the number of pixels in the component is sufficiently large, add it to our
		# mask of "large" blobs
		if numPixels > 300 and numPixels < 1500:
			mask = cv2.add(mask, labelMask)

		# show the label mask
		fn = os.path.join('images', 'labels', os.path.splitext(os.path.basename(fname))[0], 'label_%.3d.jpg' % i)
		fnames.append(fn)
		fn = os.path.join(out_path, fn)
		if not os.path.exists(os.path.dirname(fn)):
			os.makedirs(os.path.dirname(fn))
		cv2.imwrite(fn, imutils.resize(labelMask, width=160))

	# show the large components in the image
	fn = os.path.join('images', 'labels', os.path.basename(fname))
	fnames.insert(0, fn)
	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(mask, width=160))

	print '%d files' % len(fnames)

	json.dump({
		'files': fnames,
		'title': os.path.splitext(os.path.basename(__file__))[0],
	}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)

for fn in glob.glob(img_path):

	process_image(fn)

	break
