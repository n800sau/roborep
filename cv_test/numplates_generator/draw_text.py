#!/usr/bin/env python3

import os, sys, json, random
import cv2
import numpy as np

OUTDIR = 'source'
IMGDIR = 'images'
JFNAME = 'numplates.json'

for dname in (os.path.join(OUTDIR, IMGDIR),):
	if not os.path.exists(dname):
		os.makedirs(dname)

W = 600
H = int(W/1.4142)

 
blue = (255, 0, 0)
red = (0, 0, 255)
green = (0, 255, 0)
violet = (180, 0, 180)
yellow = (0, 180, 180)
white = (255, 255, 255)

font = cv2.FONT_HERSHEY_COMPLEX
fontScale = 7
thickness = 25

jdata = []

def make_numplate(num):

	image = np.full((H, W, 3), white).astype(np.uint8)
	shape = image.shape
	print(shape)

	hspace = 10
	tsize = []
	xsize = 0
	ysize = 0
	for c in str(num):

		textSize = cv2.getTextSize(c, font, fontScale, thickness);
		print(c, 'textSize(w,h)', textSize)
		tsize.append(textSize)

		xsize += textSize[0][0] + hspace
		ysize = max(ysize, textSize[0][1])

	xpos = (shape[1] - xsize) // 2
	ypos = (shape[0] + ysize) // 2
	ann = []
	for i,c in enumerate(str(num)):

		test_image = np.zeros(image.shape[:2])
		cv2.putText(test_image, c, (xpos, ypos), font, fontScale, 255, thickness)
		indeces = np.nonzero(test_image)
		x1 = min(indeces[1])
		x2 = max(indeces[1])
		y1 = min(indeces[0])
		y2 = max(indeces[0])
		w = x2-x1
		h = y2-y1
#		print('x:', x1, x2)
#		print('y:', y1, y2)
#		print('w:', w)
#		print('h:', h)

		ann.append({
			'xn': ';'.join([str(v) for v in (x1, x2, x2, x1)]),
			'yn': ';'.join([str(v) for v in (y1, y1, y2, y2)]),
			'class': c,
		})

		cv2.putText(image, c, (xpos, ypos), font, fontScale, blue, thickness)


		cv2.rectangle(image, (x1, y1), (x2, y2), green, 2)

		xpos += textSize[0][0] + hspace

	ofname = os.path.join(IMGDIR, '%d.png' % num)


	cv2.imwrite(os.path.join(OUTDIR, ofname), image)

	return {
		'filename': ofname,
		'annotations': ann,
		'class': 'image',
	}


#cv2.imshow("red panda", image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

for i in range(100):
	jdata.append(make_numplate(random.randint(100, 999)))
json.dump(jdata, open(os.path.join(OUTDIR, JFNAME), 'w'), indent=4)

print('Finished')
