#!/usr/bin/env python

import os, glob, json, time, operator
import cv2
import imutils
import numpy as np
from scipy.spatial import distance as dist

time_mark = int(time.time())

img_path = os.path.join(os.path.expanduser('~/sshfs/asus/root/sdb1/garage'), '*.jpg')
out_path = os.path.join(os.path.dirname(__file__), 'output')

conf = json.load(file('conf.json'))

fdata = {}

hrefdata = {}
fnames1 = []
fnames2 = []

def get_href(fname):
	return os.path.join('images',
		os.path.basename(os.path.dirname(fname)),
		os.path.basename(fname)
	)

def process_image(fname):

	rs = True

	image = cv2.imread(fname)


	# extract the mean and standard deviation from each channel of the
	# BGR image, then update the index with the feature vector
	(means, stds) = cv2.meanStdDev(image)
	features = list(np.concatenate([means, stds]).flatten())
	fdata[os.path.basename(fname)] = features

fcmp = []

def compare_images(fname1, fname2):
	fn1 = os.path.basename(fname1)
	fn2 = os.path.basename(fname2)
	if fn1 not in fdata:
		process_image(fname1)
	if fn2 not in fdata:
		process_image(fname2)
	d = dist.euclidean(fdata[fn1], fdata[fn2])
	fcmp.append([d, fn1, fn2])
	return d


img_start = -201
img_count = 200

lfiles = glob.glob(img_path)[img_start:img_start+img_count]

i = 0
fn = os.path.join(os.path.dirname(img_path), 'nocar', 'nocar.jpg')
for fn1 in lfiles:
	val = compare_images(fn, fn1)
	fname = 'frame_%04d.png' % i
	if val <= 20:
		# no car
		dfname = os.path.join(out_path, 'images', 'nocar', fname)
		if os.path.exists(dfname):
			os.unlink(dfname)
		image = cv2.imread(fn1)
		cv2.imwrite(dfname, imutils.resize(image, width=160))
		href = get_href(dfname)
		fnames1.append(href)
		i += 1
		hrefdata[href] = val

fn = os.path.join(os.path.dirname(img_path), 'car', 'car.jpg')
for fn1 in lfiles:
	val = compare_images(fn, fn1)
	fname = 'frame_%04d.png' % i
	if val <= 20:
		# car
		dfname = os.path.join(out_path, 'images', 'car', fname)
		if os.path.exists(dfname):
			os.unlink(dfname)
		image = cv2.imread(fn1)
		cv2.imwrite(dfname, imutils.resize(image, width=160))
		href = get_href(dfname)
		fnames2.append(href)
		i += 1
		hrefdata[href] = val

fcmp.sort()

flen1 = len(fnames1)
flen2 = len(fnames2)

if flen1 < flen2:
	fnames1 += [None] * (flen2 - flen1)
elif flen2 < flen1:
	fnames2 += [None] * (flen1 - flen2)

fnames = zip(fnames1, fnames2)

print fnames

json.dump({
	'data': hrefdata,
	'cmp': fcmp,
	'files': reduce(operator.add, fnames, ()),
	'title': os.path.splitext(os.path.basename(__file__))[0],
	'time_mark': time_mark,
}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)

#else:
#	print 'Not done. Found %d and %d' % (fnames1, fnames2)
