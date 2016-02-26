#!/usr/bin/env python

import os, glob, json, time, random
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
from skimage import feature
from skimage import exposure
import numpy as np

ROOT_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')

def find_orig_file(fn):
	#00D6FB009223(n800sau)_1_20160217173337_26871.jpg
	ts = fn.split('_')[2]
	dt = ts[:8]
	dr = '%s-%s-%s' % (dt[:4], dt[4:6], dt[6:])
	return os.path.join(ROOT_PATH, dr, fn)

def car_crop(image):
	return image
#	return image[90:, 70:270]

def process_image(fname, mask=None):
	bname = os.path.basename(fname)
	image = cv2.imread(fname)
	H = _process_image(image)
#	(H, hogImage) = _process_image(image)
#	hogImage = exposure.rescale_intensity(hogImage, out_range=(0, 255))
#	hogImage = hogImage.astype("uint8")
#	write_image(hogImage, bname + '_hog.png', fnames)
	json.dump(list(H), file(os.path.join('hog', bname + '.hog'), 'w'))
	return H

def _process_image(image):
	image = car_crop(image)
#	write_image(image, bname + '_image.png', fnames)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	return feature.hog(gray, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), normalise=True, visualise=False)
#	return feature.hog(gray, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), normalise=True, visualise=True)

if __name__ == '__main__':

	time_mark = int(time.time())

	img_path = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-02-21')
	out_path = os.path.join(os.path.dirname(__file__), 'output')

	SAMPLE_FNAME = 'sample.json'

	if os.path.exists(SAMPLE_FNAME):
		flist = json.load(file(SAMPLE_FNAME))
	else:
		flist = glob.glob(os.path.join(img_path, '*.jpg'))
		json.dump(flist, file(SAMPLE_FNAME, 'w'), indent=2)

	i = 0
	for fn in flist:
		frame = cv2.imread(fn)
		i += 1
		process_image(fn)

