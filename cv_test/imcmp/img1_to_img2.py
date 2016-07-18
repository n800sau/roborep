#!/usr/bin/env python

import os, json
import cv2
import imutils
from matplotlib import pyplot as plt

in_dir = os.path.join('data', 'images1')
out_dir = os.path.join('data', 'images2')
out_hist_dir = os.path.join('data', 'images3')

def process_image(fname, in_dir, out_dir, out_his_dir):

	image = cv2.imread(os.path.join(in_dir, fname))

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(image)
	hist = cv2.calcHist([h], [0], None, [256], [0, 256])
#	hist /= hist.max()

	fig = plt.figure(figsize=(4.5, 1.8))
	ax = fig.add_subplot(111)
	ax.set_title("Histogram")
	ax.set_xlabel("Bins")
	ax.set_ylabel("# of Pixels")
#	ax.set_ylim([0, 300])
	ax.plot(hist)
	ax.set_xlim([0, 256])
	plt.savefig(os.path.join(out_hist_dir, fname), dpi = (50))
	plt.close(fig)

#	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# apply a series of erosions
	for i in xrange(5, 10, 5):
		eroded = cv2.erode(h.copy(), None, iterations=i + 1)
		cv2.imwrite(os.path.join(out_dir, fname), eroded)

for root, dirs, files in os.walk(in_dir, followlinks=True):

	for fname in files:
		process_image(fname, in_dir, out_dir, out_hist_dir)

	break

print 'Finished'
