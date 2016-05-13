#!/usr/bin/env python

import sys, os, json, glob
from sklearn.svm import SVC
import cv2
from skimage import feature
import cPickle
from imutils import paths

BASEPATH = 'data'
TESTPATH = os.path.join(BASEPATH, 'testing')
MODELPATH = os.path.join(BASEPATH, 'model.svc')

model = cPickle.loads(open(MODELPATH).read())

print 'Labels:', model.classes_

for dname in glob.glob(os.path.join(TESTPATH, '*')):
	if os.path.isdir(dname):

		bdname = os.path.basename(dname)

		# loop over the testing images
		for testingPath in paths.list_images(dname):
			# load the image and make predictions
			image = cv2.imread(testingPath)
			image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			hist = feature.hog(image,
				orientations=9,
				pixels_per_cell=[4, 4],
				cells_per_block=[2, 2],
				transform_sqrt=True)
			hist[hist < 0] = 0
			labels = model.predict_proba(hist)
			labels = zip(model.classes_, labels[0])
			labels.sort(key=lambda x: x[1], reverse=True)
			if abs(labels[0][1] - labels[1][1]) > 0.5:
				outpath = os.path.join(BASEPATH, 'output', bdname, labels[0][0])
			else:
				outpath = os.path.join(BASEPATH, 'output', bdname, '_')
			if not os.path.exists(outpath):
				os.makedirs(outpath)
			ofname = os.path.join(outpath, os.path.basename(testingPath))
			cv2.imwrite(ofname, image)
