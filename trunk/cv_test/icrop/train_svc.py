#!/usr/bin/env python

import sys, os, json, glob
from sklearn.svm import SVC
import cv2
from skimage import feature
import cPickle

BASEPATH = 'data'
LABELSPATH = os.path.join(BASEPATH, 'labels')
MODELPATH = os.path.join(BASEPATH, 'model.svc')

all_labels = []
data = []
labels = []
for lpath in glob.glob(os.path.join(LABELSPATH, '*')):
	if os.path.isdir(lpath):
		lname = os.path.basename(lpath)
		for fname in glob.glob(os.path.join(lpath, '*')):
			if os.path.isfile(fname):
				image = cv2.imread(fname)
				image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
				hist = feature.hog(image,
					orientations=9,
					pixels_per_cell=[4, 4],
					cells_per_block=[2, 2],
					transform_sqrt=True)
				hist[hist < 0] = 0
				data.append(hist)
				labels.append(lname)

model = SVC(kernel="rbf", C=10.0, gamma=0.001, random_state=84, probability=True)
model.fit(data, labels)

f = open(MODELPATH, "w")
f.write(cPickle.dumps(model))
f.close()
