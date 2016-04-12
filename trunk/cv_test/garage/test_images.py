#!/usr/bin/env python

import os, glob, json, random
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
import numpy as np
from matplotlib import pyplot as plt
from make_hogs import find_orig_file

img_path1 = os.path.join(os.path.expanduser('traindata/car_inside'), '*.jpg')
img_path2 = os.path.join(os.path.expanduser('traindata/nocar'), '*.jpg')
out_path = os.path.join(os.path.dirname(__file__), 'output')

fnames = []


def write_image(image, fname, fnames):

	fn = os.path.join('images', fname)
	fnames.append(fn)
	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(image, width=160))


def process_image(fname):

	plt.ioff()

	image = cv2.imread(fname)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	image = cv2.imread(fname)
	labimage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	h,s,v = cv2.split(labimage)

	hist = cv2.calcHist([h], [0], None, [256], [0, 256])

	# plot the histogram
	plt.figure(figsize=(2,2))
	plt.title("Hue Histogram")
	plt.xlabel("Bins")
	plt.ylabel("# of Pixels")
	plt.plot(hist)
	plt.xlim([0, 256])

	fn = os.path.join('images', 'hue_histogram.png')
	fnames.append(fn)
	plt.savefig(os.path.join(out_path, fn), bbox_inches='tight', format='png')


	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	image = cv2.GaussianBlur(image, (11, 11), 0)

	write_image(image, 'blurred.jpg', fnames)

	mask = (image > 170).astype("uint8")
	cv2.bitwise_and(image, image, mask = mask)



	# construct a grayscale histogram
	hist = cv2.calcHist([image], [0], None, [256], [0, 256])


	# plot the histogram
	plt.figure(figsize=(2,2))
	plt.title("Grayscale Histogram")
	plt.xlabel("Bins")
	plt.ylabel("# of Pixels")
	plt.plot(hist)
	plt.xlim([0, 256])

	fn = os.path.join('images', 'histogram.png')
	fnames.append(fn)
	plt.savefig(os.path.join(out_path, fn), bbox_inches='tight', format='png')

	# normalize the histogram
	hist /= hist.sum()

	# plot the normalized histogram
	plt.figure(figsize=(2,2))
	plt.title("Grayscale Histogram (Normalized)")
	plt.xlabel("Bins")
	plt.ylabel("% of Pixels")
	plt.plot(hist)
	plt.xlim([0, 256])


	fn = os.path.join('images', 'histogram_norm.png')
	fnames.append(fn)
	plt.savefig(os.path.join(out_path, fn), bbox_inches='tight', format='png')
#	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(mask, width=160))

	print '%d files' % len(fnames)

	json.dump({
		'files': fnames,
		'title': os.path.splitext(os.path.basename(__file__))[0],
	}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)

flist = random.sample(glob.glob(img_path1), 1) + random.sample(glob.glob(img_path2), 1)

for fn in flist:

	fn = find_orig_file(os.path.basename(fn))
	print fn
	process_image(fn)
