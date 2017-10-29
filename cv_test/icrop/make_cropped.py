#!/usr/bin/env python

import os, cv2, json

BASEPATH = 'data'
DRAWERS = os.path.join(BASEPATH, 'drawers')
IMAGES = os.path.join(BASEPATH, 'images')
OUTPUT = os.path.join(BASEPATH, 'cropped')

# unknown coef
coef = 1920 / 200.
for root, dirs, fnames in os.walk(DRAWERS, followlinks=True):
	for fname in fnames:
		name,ext = os.path.splitext(fname)
		if ext == '.json':
			ifname = os.path.join(IMAGES, name)
			iname,iext = os.path.splitext(name)
			image = cv2.imread(ifname)
			for i, area in enumerate(json.load(file(os.path.join(root, fname)))):
				cropped = image[area['y1']*coef:area['y2']*coef, area['x1']*coef:area['x2']*coef]
				cfname = os.path.join(OUTPUT, '%s.%d%s' % (iname, i, iext))
				print(cfname, cropped.shape)
				cv2.imwrite(cfname, cropped)
