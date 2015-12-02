#!/usr/bin/env python

import os, glob, json
import cv2
import imutils

img_path = os.path.join(os.path.expanduser('~/sshfs/asus/root/sdb1/garage'), '*.jpg')
out_path = os.path.join(os.path.dirname(__file__), 'output')

def process_image(fname):

	image = cv2.imread(fname)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# apply a series of erosions
	fnames = []
	for i in xrange(5, 10, 5):
		eroded = cv2.erode(gray.copy(), None, iterations=i + 1)
		fn = os.path.join('images', 'erosion_%d.png' % i)
		fnames.append(fn)
		cv2.imwrite(os.path.join(out_path, fn), imutils.resize(eroded, width=160))
	json.dump({
		'files': fnames,
		'title': os.path.splitext(os.path.basename(__file__))[0],
	}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)

for fn in glob.glob(img_path):

	process_image(fn)

	break
