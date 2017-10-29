#!/usr/bin/env python

import os, sys, imutils, glob, cv2, shutil

SUBDIR = '2016-09-24'
#SRC_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
SRC_PATH = os.path.expanduser('~/work/garage')
DST_PATH = os.path.expanduser('output/images/thumbs')

def process(subdir):
	srcdir = os.path.join(SRC_PATH, subdir)
	for fn in glob.glob(os.path.join(srcdir, '*')):
		bname = os.path.basename(fn)
		if os.path.isdir(fn):
			process(os.path.join(subdir, bname))
		else:
			if os.path.splitext(fn)[-1] == '.jpg':
				dfn = os.path.join(DST_PATH, subdir, bname)
				ddn = os.path.dirname(dfn)
				if not os.path.exists(ddn):
					os.makedirs(ddn)
				image = cv2.imread(fn)
				image = imutils.resize(image, width=40)
				cv2.imwrite(dfn, image)

if __name__ == '__main__':

	for fn in glob.glob(os.path.join(SRC_PATH, SUBDIR, '*')):
		bname = os.path.basename(fn)
		if os.path.isdir(fn):
			process(os.path.join(SUBDIR, bname))
