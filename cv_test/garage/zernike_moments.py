#!/usr/bin/env python

import os, glob, json, time
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
import numpy as np

time_mark = int(time.time())

IMG_WIDTH = 320

conf = json.load(file('conf.json'))

def apply_mask(image):
	global conf
	mask = np.zeros(image.shape[:2], dtype="uint8")
	mask[mask.shape[0] * conf['mask']['y1']:mask.shape[0] * conf['mask']['y2'], mask.shape[1] * conf['mask']['x1']:mask.shape[1] * conf['mask']['x2']] = 255
	return cv2.bitwise_and(image, image, mask = mask)

ctypes = ['car', 'nocar', 'car_with_cars', 'nocar_with_cars']



base_path = os.path.expanduser('~/sshfs/asus/root/sdb1/garage')

def get_features(fpath):
	rs = None
	for fname in glob.glob(os.path.join(fpath, '*.jpg')):
		frame = cv2.imread(fname)
		frame = apply_mask(frame)
		frame = imutils.resize(frame, width=IMG_WIDTH)
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)
#		gray = cv2.equalizeHist(gray)
		if rs is None:
			rs = gray.copy().astype("float")
		else:
			cv2.accumulateWeighted(gray, rs, 0.5)
	return rs

avgmap = {}
for ct in ctypes:
	#make background averages
	avgmap[ct] = collect_avg(os.path.join(base_path, ct))

img_path = os.path.expanduser('~/sshfs/asus/root/sdb1/garage/2015-12-02')
out_path = os.path.join(os.path.dirname(__file__), 'output')

def write_image(image, fname, fnames, data, value):

	fn = os.path.join('images', fname)
	fnames.append(fn)
	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(image, width=160))
	data[fn] = value


def find_changes(bg_avg, fname):

	rs = None

	frame = cv2.imread(fname)
	frame = apply_mask(frame)

	frame = imutils.resize(frame, width=IMG_WIDTH)

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	gray = cv2.GaussianBlur(gray, (21, 21), 0)

#	gray = cv2.equalizeHist(gray)

	frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(bg_avg))

	thresh = cv2.threshold(frameDelta, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

	thresh = cv2.dilate(thresh, None, iterations=2)

	n = np.sum(thresh)

	print n

	if n > 1000000:
#	(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#	for c in cnts:
#		area = cv2.contourArea(c)
		# if the contour is too small, ignore it
#		if area < conf["min_area"]:
#			continue

#		cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
#		(x, y, w, h) = cv2.boundingRect(c)

#		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

		rs = thresh

	return rs


fnames = []
data = {}

img_start = 0
img_count = 500
n = 5
for fn in glob.glob(os.path.join(img_path, '*.jpg'))[img_start:img_start+img_count]:

	for ct in ctypes:
		changes = find_changes(avg_car, fn)

	nocar_change = find_changes(avg_nocar, fn)
	if (not nocar_change is None) and (not car_change is None):
		for (frame,title) in ((nocar_change, 'nocar'), (car_change, 'car')):
			if not frame is None:
				fname = 'frame_%04d.png' % (len(fnames)+1)
				write_image(frame, fname, fnames, data, title)
				n -= 1
	if n <= 0:
		break

print '%d files' % len(fnames)

json.dump({
	'files': fnames,
	'title': os.path.splitext(os.path.basename(__file__))[0],
	'time_mark': time_mark,
	'data': data,
}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)
