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

def process_image(fname):

	image = cv2.imread(fname)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	fnames = []

	image = cv2.imread(fname)
	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# construct a grayscale histogram
	hist = cv2.calcHist([image], [0], None, [256], [0, 256])

	plt.ioff()

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
		'time_mark': time_mark,
	}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)

for fn in glob.glob(img_path)[:10]:

	process_image(fn)

	break
