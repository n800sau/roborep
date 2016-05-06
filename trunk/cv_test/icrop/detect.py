#!/usr/bin/env python

from __future__ import print_function
import os, dlib, json, cv2, glob
from imutils import paths
from skimage import io

BASEPATH = 'data'

print("[INFO] reading classifier from file...")
detector = dlib.simple_object_detector(os.path.join(BASEPATH, 'detector', 'simple.svm'))

fnames = []

for dname in glob.glob(os.path.join(BASEPATH, 'testing', '2016-*-*')):
	if os.path.isdir(dname):

		bdname = os.path.basename(dname)

		# loop over the testing images
		for testingPath in paths.list_images(dname):
			# load the image and make predictions
			image = cv2.imread(testingPath)
			boxes = detector(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

			outpath = os.path.join(BASEPATH, 'output', bdname, 'no' if len(boxes) == 0 else 'yes')

			# loop over the bounding boxes and draw them
			for b in boxes:
				(x, y, w, h) = (b.left(), b.top(), b.right(), b.bottom())
				print(x, y, w, h)
				cv2.rectangle(image, (x, y), (w, h), (0, 255, 0), 2)

			# show the image
			if not os.path.exists(outpath):
				os.makedirs(outpath)
			ofname = os.path.join(outpath, os.path.basename(testingPath))
			cv2.imwrite(ofname, image)

print("[INFO] finished")
