#!/usr/bin/env python

import os, glob, json, time, random
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
from skimage import feature
from skimage import exposure
import numpy as np


#BASE_DIR = 'cat5'
#ROOT_PATH = os.path.expanduser('~/work/opencv/cat')
#BASE_DIR = '2014-10_01'
BASE_DIR = '2016-09-10'
ROOT_PATH = os.path.expanduser('~/work/garage')
#ROOT_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
#ROOT_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/n800s/mydvd/pics_sony')

def find_orig_file(fn):
	#00D6FB009223(n800sau)_1_20160217173337_26871.jpg
	ts = fn.split('_')[2]
	dt = ts[:8]
	dr = '%s-%s-%s' % (dt[:4], dt[4:6], dt[6:])
	return os.path.join(ROOT_PATH, dr, fn)

def car_mask(image):
# to use car root area only
	mask = np.zeros(image.shape[:2], dtype="uint8")
	cv2.rectangle(mask, (int(image.shape[1]*.09), int(image.shape[0]*.35)), (int(image.shape[1]*.7), int(image.shape[0])), 255, -1)
	return mask

def process_image(fname, mask_proc=None):
	H = None
	print fname,
	bname = os.path.basename(fname)
	print '>', bname,
	image = cv2.imread(fname)
	if image is None:
		print 'Error: Image can not be read.'
	else:
		print ',', image.shape
		if image.shape[0] * image.shape[1] > 640 * 480:
			image = imutils.resize(image, height=480)
		(H, hogImage) = _process_image(image)
#	(H, hogImage) = _process_image(image)
#	hogImage = exposure.rescale_intensity(hogImage, out_range=(0, 255))
#	hogImage = hogImage.astype("uint8")
#	write_image(hogImage, bname + '_hog.png', fnames)
#		np.savetxt(file(os.path.join('hog', bname + '.hog'), 'w'), H)
		print H.shape
		json.dump(list(H), file(os.path.join('hog', bname + '.hog'), 'w'))
	return H

def _process_image(image, mask_proc=None):
	if image is None:
		print 'Error: Image is None.'
	else:
		if mask_proc:
			mask = mask_proc(image)
			image = cv2.bitwise_and(image, image, mask=mask)
#		write_image(image, bname + '_image.png', fnames)
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		return feature.hog(gray, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), visualise=True)
#		return feature.hog(gray, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), normalise=True, visualise=True)

if __name__ == '__main__':

	time_mark = int(time.time())

	img_path = os.path.join(ROOT_PATH, BASE_DIR)
	out_path = os.path.join(os.path.dirname(__file__), 'output')

	SAMPLE_FNAME = 'sample.json'

#	if os.path.exists(SAMPLE_FNAME):
#		flist = json.load(file(SAMPLE_FNAME))
#	else:
	flist = glob.glob(os.path.join(img_path, '*.jpg'))
	json.dump(flist, file(SAMPLE_FNAME, 'w'), indent=2)

	i = 0
	for fn in flist:
		frame = cv2.imread(fn)
		i += 1
		process_image(fn, mask_proc=car_mask)

