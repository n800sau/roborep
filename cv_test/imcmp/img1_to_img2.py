#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

from paths import r_list_dirs, list_images
import json
import cv2
import imutils
from matplotlib import pyplot as plt

in_dir = os.path.join('data', 'images1')
out_dir = os.path.join('data', 'images2')
out_hist_dir = os.path.join('data', 'images3')

def process_image(fname, in_dir, out_dir, out_hist_dir):

	for dr in (out_dir, out_hist_dir):
		dr = os.path.dirname(os.path.join(dr, fname))
		if not os.path.exists(dr):
			os.makedirs(dr)

	image = cv2.imread(os.path.join(in_dir, fname))

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(hsv)
#	lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
#	l,a,b = cv2.split(lab)

	eroded = cv2.erode(h.copy(), None, iterations=10)
	eroded = cv2.merge([eroded, s, v])
	eroded = cv2.cvtColor(eroded, cv2.COLOR_HSV2BGR)
	cv2.imwrite(os.path.join(out_dir, fname), eroded)


	hist = cv2.calcHist([eroded], [0], None, [256], [0, 256])
	hist /= hist.max()

	fig = plt.figure(figsize=(4.5, 1.8))
	ax = fig.add_subplot(111)
	ax.set_title("Histogram")
	ax.set_xlabel("Bins")
	ax.set_ylabel("# of Pixels")
#	ax.set_ylim([0, 300])
	ax.plot(hist)
	ax.set_xlim([0, 256])
#	print os.path.join(out_hist_dir, fname)
	plt.savefig(os.path.join(out_hist_dir, fname), dpi = (50))
	plt.close(fig)

#	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


for subdir in r_list_dirs(in_dir):

	for fname in list_images(in_dir, subdir):
#		print in_dir, dpath, fname
		process_image(fname, os.path.join(in_dir, subdir), os.path.join(out_dir, subdir), os.path.join(out_hist_dir, subdir))

print 'Finished'
