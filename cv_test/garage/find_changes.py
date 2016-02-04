#!/usr/bin/env python

import os, glob, json, time
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
import numpy as np

time_mark = int(time.time())

img_path = os.path.join(os.path.expanduser('~/sshfs/asus/root/sdb1/garage/2015-12-02'), '*.jpg')
out_path = os.path.join(os.path.dirname(__file__), 'output')

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)

	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)

	# return the edged image
	return edged


def write_image(image, fname, fnames):

	fn = os.path.join('images', fname)
	fnames.append(fn)
	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(image, width=160))


conf = json.load(file('conf.json'))

fnames = []

def process_images(fname1, fname2):

	rs = True

	image1 = cv2.imread(fname1)
	image2 = cv2.imread(fname2)

	image1 = imutils.resize(image1, width=320)
	image2 = imutils.resize(image2, width=320)

	gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
	gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

	gray1 = cv2.GaussianBlur(gray1, (7, 7), 0)
	gray2 = cv2.GaussianBlur(gray2, (7, 7), 0)

#	gray1 = cv2.equalizeHist(gray1)
#	gray2 = cv2.equalizeHist(gray2)

	imdiff = cv2.subtract(gray1, gray2)

	imdiff[imdiff < 10] = 0

	print 'A:',np.min(imdiff), np.max(imdiff)

	(T, processed) = cv2.threshold(imdiff, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

#	processed = cv2.adaptiveThreshold(imdiff, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 25, 15)


	rs = np.sum(processed > 0) > 1000

	print 'B:',np.min(processed), np.max(processed), np.sum(processed > 0)

	if rs:

		h,w,n = image1.shape
		cimage = np.zeros((h, w * 2, n), dtype="uint8")
		cimage[:,0:w,:] = image1[::]
		cimage[:,w:,:] = image2[::]

		write_image(cimage, 'frame_%d.png' % (len(fnames)+1), fnames)
		write_image(processed, 'frame_%d.png' % (len(fnames)+1), fnames)

#	gray1 = cv2.GaussianBlur(gray1, (7, 7), 0)
#	gray1 = cv2.medianBlur(gray1, 21)

#	gray1 = cv2.adaptiveThreshold(gray1, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 25, 15)

#	gray1 = auto_canny(gray1)

#	for k in range(5):

#		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1+k*5, 1+k*5))
#		processed = cv2.morphologyEx(gray1, cv2.MORPH_OPEN, kernel)
#		write_image(processed, 'frame_%d.png' % (len(fnames)+1), fnames)

	return rs


img_start = 0
img_count = 500
fname1 = None
n = 5
for fn in glob.glob(img_path)[img_start:img_start+img_count]:

	if not fname1 is None:
		if process_images(fname1, fn):
			n -= 1
			if n <= 0:
				break
	fname1 = fn

print '%d files' % len(fnames)

json.dump({
	'files': fnames,
	'title': os.path.splitext(os.path.basename(__file__))[0],
	'time_mark': time_mark,
}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)
