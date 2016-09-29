#!/usr/bin/env python

import os, sys, imghdr
from PIL import Image

BASEPATH = 'birds'

def test_image(fname):
	rs = False
	image = Image.open(fname)
	try:
		image.load()
		rs = True
	except IOError as e:
		print 'Bad image: %s' % e
		os.remove(fname)
	return rs

for root, dirs, files in os.walk(BASEPATH, followlinks=True):
	for fname in files:
		fname = os.path.join(os.path.join(root, fname))
		try:
			itype = imghdr.what(fname)
			if itype is None:
				os.remove(fname)
			else:
				if test_image(fname):
					print itype
		except Exception, e:
			print e
