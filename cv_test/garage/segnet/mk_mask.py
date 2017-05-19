#!/usr/bin/env python

import os, sys, json
import Image
import ImageDraw
import cv2
import numpy as np

imlist = json.load(file('garage.json'))

IMGDIR = 'data/images'
OUTPUT = 'data/annotations'
TRAIN_SIZE = (480, 360)

bg_index = 0

clabels = {
	'car': 1,
#	'car': 128,
#	'car': (255, 255, 0),
}

weights = dict([(v, {'pixels': 0, 'total': 0}) for k,v in clabels.items()])
weights[bg_index] = {'pixels': 0, 'total': 0}

for im in imlist:
	if im['annotations']:
		fn = im['filename']
		image = Image.open(fn)
		image = np.asarray(image)
		image = cv2.resize(image, TRAIN_SIZE, interpolation=cv2.INTER_CUBIC)
		image = Image.fromarray(image)
#		image.transform(TRAIN_SIZE, Image.EXTENT, (0,0,x2,y2))
#		image.resize(TRAIN_SIZE, Image.ANTIALIAS)
		width, height = image.size
		annimage = Image.new('L', image.size)
		pdraw = ImageDraw.Draw(annimage)
		for ann in im['annotations']:
			xn = [float(v) for v in ann['xn'].split(';')]
			yn = [float(v) for v in ann['yn'].split(';')]
			poly = zip(xn, yn)
			pdraw.polygon(poly, fill=clabels[ann['class']])
		anndata = np.asarray(annimage)
		annsize = anndata.size
		bg_pixels = annsize
		for v in clabels.values():
			kn = np.count_nonzero(anndata == v)
			if kn > 0:
				bg_pixels -= kn
				weights[v]['pixels'] += kn
				weights[v]['total'] += annsize
		if bg_pixels > 0:
			weights[bg_index]['pixels'] += bg_pixels
			weights[bg_index]['total'] += annsize
		bname = os.path.basename(fn)
		annimage.save(os.path.join(OUTPUT, bname + '.png'))
		image.save(os.path.join(IMGDIR, bname + '.png'))
for k,v in weights.items():
	v['freq'] = float(v['pixels']) / v['total']
mfreq = np.median([v['freq'] for v in weights.values()])
print json.dumps(weights, indent=2)
print 'mfreq:', mfreq
f = file('weights.txt', 'w')
for ki,v in sorted(list(weights.items()), key=lambda v: v[0]):
	f.write('class_weighting: %s\n' % (mfreq / v['freq']))
f.close()
