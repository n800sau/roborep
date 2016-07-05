#!/usr/bin/env python

import sys, os, json, glob
from sklearn.svm import SVC
import cv2
from skimage import feature
from sklearn.externals import joblib
#import cPickle
import pickle

BASEPATH = 'data'
LABELSPATH = os.path.join(BASEPATH, 'labels')
MODELPATH = os.path.join(BASEPATH, 'detector', 'model.svc')

all_labels = []
data = []
labels = []
for lpath in glob.glob(os.path.join(LABELSPATH, '*')):
	if os.path.isdir(lpath):
		lname = os.path.basename(lpath)
		for fname in glob.glob(os.path.join(lpath, '*')):
			if os.path.isfile(fname):
				image = cv2.imread(fname)
				if image is None:
					print '%s is not readable' % fname
				else:
					image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
					hist = feature.hog(image,
						orientations=9,
						pixels_per_cell=[4, 4],
						cells_per_block=[2, 2],
						transform_sqrt=True)
					hist[hist < 0] = 0
					data.append(hist)
					labels.append(lname)

#print type(data), type(labels)

model = SVC(kernel="rbf", C=10.0, gamma=0.001, random_state=84, probability=True)

model.fit(data, labels)

#pickle.dump(model, open(MODELPATH, "w"))
pickle.dump(model, open(MODELPATH, "w"), protocol=pickle.HIGHEST_PROTOCOL)
#joblib.dump(model, MODELPATH)
