#!/usr/bin/env python

import os, sys, json
import Image
import ImageDraw

imlist = json.load(file('trucks.json'))

OUTPUT = 'annotations'

clabels = {
	'dirt': 1,
	'belaz': 2,
#	'dirt': 128,
#	'belaz': 255,
#	'dirt': (255, 255, 0),
#	'belaz': (0, 255, 255),
}

for im in imlist:
	if im['annotations']:
		fn = im['filename']
		image = Image.open(fn)
		width, height = image.size
		oimage = Image.new('L', image.size)
		pdraw = ImageDraw.Draw(oimage)
		for ann in im['annotations']:
			xn = [float(v) for v in ann['xn'].split(';')]
			yn = [float(v) for v in ann['yn'].split(';')]
			poly = zip(xn, yn)
			pdraw.polygon(poly, fill=clabels[ann['class']])
		oimage.save(os.path.join(OUTPUT, os.path.basename(fn)))
