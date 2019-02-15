#!/usr/bin/env python3

import os, sys, json, random, imutils
import cv2
import numpy as np

OUTDIR = 'source'
IMGDIR = 'images'
LABELDIR = 'labels'

for dname in (os.path.join(OUTDIR, IMGDIR),os.path.join(OUTDIR, LABELDIR),):
	if not os.path.exists(dname):
		os.makedirs(dname)

W = 600
H = int(W/1.4142)

blue =   (255, 0, 0, 255)
red =    (0, 0, 255, 255)
green =  (0, 255, 0, 255)
violet = (180, 0, 180, 255)
yellow = (0, 180, 180, 255)
white =  (255, 255, 255, 255)
black =  (0, 0, 0, 255)
transparent = (0, 0, 0, 0)

font = cv2.FONT_HERSHEY_COMPLEX
fontScale = 7
thickness = 25

def rotate_bound(image, angle):
	# grab the dimensions of the image and then determine the
	# center
	(h, w) = image.shape[:2]
	(cX, cY) = (w / 2, h / 2)

	# grab the rotation matrix (applying the negative of the
	# angle to rotate clockwise), then grab the sine and cosine
	# (i.e., the rotation components of the matrix)
	M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)
	cos = np.abs(M[0, 0])
	sin = np.abs(M[0, 1])

	# compute the new bounding dimensions of the image
	nW = int((h * sin) + (w * cos))
	nH = int((h * cos) + (w * sin))

	# adjust the rotation matrix to take into account translation
	M[0, 2] += (nW / 2) - cX
	M[1, 2] += (nH / 2) - cY

	# perform the actual rotation and return the image
	# use cv2.INTER_NEAREST to avoid introducing new colour
	return cv2.warpAffine(image, M, (nW, nH), flags=cv2.INTER_NEAREST)


def make_numplate_rotate(num):

	image = np.full((H, W, 4), transparent).astype(np.uint8)
	label_image = np.zeros(image.shape[:2]).astype(np.uint8)
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

		cv2.putText(label_image, c, (xpos, ypos), font, fontScale, int(c), thickness)
		cv2.putText(image, c, (xpos, ypos), font, fontScale, blue, thickness)

		xpos += textSize[0][0] + hspace

	ofname = '%d.png' % num
	cv2.imwrite(os.path.join(OUTDIR, IMGDIR, ofname), image)
	cv2.imwrite(os.path.join(OUTDIR, LABELDIR, ofname), label_image)


	for i in range(10):
		angle = random.randint(1, 360//5) * 5
		rotated_label_image = rotate_bound(label_image, angle)
		rotated_image = rotate_bound(image, angle)
		if list(np.unique(label_image)) != list(np.unique(rotated_label_image)):
			print('uniq before:', np.unique(label_image))
			print('uniq after:', np.unique(rotated_label_image))
		# replace transparency with color
#		image[np.where((rotated_image==transparent).all(axis=2))] = green
		ofname_rotated = '%d_r%d.png' % (num, angle)

		cv2.imwrite(os.path.join(OUTDIR, IMGDIR, ofname_rotated), rotated_image)
		cv2.imwrite(os.path.join(OUTDIR, LABELDIR, ofname_rotated), rotated_label_image)

for i in range(100):
	make_numplate_rotate(random.randint(100, 999))

print('Finished')
