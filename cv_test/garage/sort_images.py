#!/usr/bin/env python

import os, sys, glob, json, time, random, shutil
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
from skimage import feature
from skimage import exposure
from sklearn.cluster import KMeans
import numpy as np
import mahotas
from make_hogs import process_image

IMG_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-02-16')

time_mark = int(time.time())

LEVELS = 3

MAX_NFILES = 1000

def fill_a_level(shf, subdir, flist):

	hog_path = 'hog'

	fnames = []

	featurelist = []

	processed = []
	for fn in flist:
		bname = os.path.basename(fn)
		hogname = os.path.join(hog_path, bname + '.hog')
		if os.path.exists(hogname):
			f = json.load(file(hogname))
		else:
			print 'Not Found !!!!!!!!!!!!!!!!!!!!!!!'
			f = process_image(fn)
		featurelist.append(f)
		processed.append(os.path.basename(fn))

	clt = KMeans(n_clusters=2)
	clt.fit(featurelist)

	slabels = set(clt.labels_)

	files = dict([(l, []) for l in slabels])

	for label,fn in zip(clt.labels_, processed):
		files[label].append(fn)

	for label,lfn in files.items():
		ddir = os.path.join(subdir, str(label))
		print >>shf, 'mkdir %s' % ddir
		for bname in lfn:
			print >>shf, 'ln \'%s\' \'%s/%s\'' % (bname, ddir, bname)
	return files

def loop_a_level(shf, subdir, levels, flist):
	for i in range(levels):
		if i == 0:
			files = fill_a_level(shf, subdir, flist)
		else:
			for l, lfn in files.items():
				loop_a_level(shf, os.path.join(subdir, str(l)), levels-1, lfn)
			break

if __name__ == '__main__':

	cmdfname = 'move_files.sh'
	shf = file(cmdfname, 'w')
	loop_a_level(shf, '', LEVELS, glob.glob(os.path.join(IMG_PATH, '*.jpg'))[:MAX_NFILES])
	shf.close()
	shutil.copy(cmdfname, os.path.join(IMG_PATH, cmdfname))