#!/usr/bin/env python

import sys, os, json, glob
import progressbar
from sklearn.svm import SVC
import cv2
from skimage import feature
#import cPickle
from sklearn.externals import joblib
from imutils import paths

BASEPATH = 'data'
TESTPATH = os.path.join(BASEPATH, 'testing')
MODELPATH = os.path.join(BASEPATH, 'detector', 'model.svc')

def detect_label(model, image):
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
	print labels
	return labels[0][0] if abs(labels[0][1] - labels[1][1]) > 0.5 else '_'

#model = cPickle.loads(open(MODELPATH).read())

model = joblib.load(MODELPATH)

print 'Labels:', model.classes_

for dname in glob.glob(os.path.join(TESTPATH, '*')):
	if os.path.isdir(dname):

		bdname = os.path.basename(dname)

		flist = list(paths.list_images(dname))

		widgets = ['%s: ' % bdname, progressbar.Percentage(), " ", progressbar.Bar(), " ", progressbar.ETA()]
		pbar = progressbar.ProgressBar(maxval=len(flist), widgets=widgets).start()
		# loop over the testing images
		for i, testingPath in enumerate(flist):
			# load the image and make predictions
			image = cv2.imread(testingPath)
			label = detect_label(model, image)
			outpath = os.path.join(BASEPATH, 'output', bdname, label)
			if not os.path.exists(outpath):
				os.makedirs(outpath)
			ofname = os.path.join(outpath, os.path.basename(testingPath))
			cv2.imwrite(ofname, image)
			pbar.update(i)

