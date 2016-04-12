#!/usr/bin/env python

import os, sys, glob, json
from sklearn.cross_validation import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.svm import SVC
from sklearn.metrics import classification_report
from sklearn.externals import joblib
import numpy as np
import cv2
import imutils
from make_hogs import _process_image, find_orig_file, process_image, car_mask

BASEPATH = 'traindata'
HOGPATH = 'hog'

def get_data():
	rs = []
	for ln in glob.glob(os.path.join(BASEPATH, '*')):
		if os.path.isdir(ln):
			label = os.path.basename(ln)
			for fn in glob.glob(os.path.join(ln, '*.jpg')):
				v = os.path.basename(fn)
				rs.append((label, v))
	return rs

def get_hog_data():
	data = get_data()
	ds = []
	ls = []
	for l,n in data:
		hf = os.path.join(HOGPATH, n + '.hog')
		orig_fname = find_orig_file(n)
		print 'orig fname:', orig_fname
		if not os.path.exists(hf):
			if process_image(orig_fname, mask_proc=car_mask) is None:
				# skip the image
				continue
		hog = json.load(file(hf))
		ds.append(hog)
		ls.append(l)
	ds = np.array(ds)
	return {'data': ds, 'labels': ls}

if __name__ == '__main__':


#	DST_PATH = os.path.expanduser('output/images/predict')

	hdata = get_hog_data()

#	trainData = hdata['data']
#	trainLabels = hdata['labels']

	(trainData, testData, trainLabels, testLabels) = train_test_split(hdata['data'], hdata['labels'], test_size=0.40, random_state=42)

	(trainData, valData, trainLabels, valLabels) = train_test_split(trainData, trainLabels, test_size=0.1, random_state=84)

	# initialize the values of k for our k-Nearest Neighbor classifier along with the
	# list of accuracies for each value of k
	kVals = range(1, 30, 2)
#	accuracies = []

	# loop over various values of `k` for the k-Nearest Neighbor classifier
#	for k in xrange(1, 30, 2):
		# train the k-Nearest Neighbor classifier with the current value of `k`
#		model = KNeighborsClassifier(n_neighbors=k)
#		model.fit(trainData, trainLabels)

		# evaluate the model and update the accuracies list
#		score = model.score(valData, valLabels)
#		print("k=%d, accuracy=%.2f%%" % (k, score * 100))
#		accuracies.append(score)

	# find the value of k that has the largest accuracy
#	i = np.argmax(accuracies)
	i = 1
#	print("k=%d achieved highest accuracy of %.2f%% on validation data" % (kVals[i], accuracies[i] * 100))

	# re-train our classifier using the best k value and predict the labels of the
	# test data
#	model = SVC(kernel="linear")
	model = LogisticRegression()
#	model = KNeighborsClassifier(n_neighbors=kVals[i])
	model.fit(trainData, trainLabels)

	mfname = 'models/knc.pkl'
#	model = joblib.load(mfname)

	# evaluate the model and update the accuracies list
	accuracy = model.score(valData, valLabels) * 100
	print("accuracy=%.2f%%" % (accuracy,))

	joblib.dump(model, mfname)
