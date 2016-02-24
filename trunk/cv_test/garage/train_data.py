#!/usr/bin/env python

import os, sys, glob, json
from sklearn.cross_validation import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import classification_report
from sklearn.externals import joblib
import numpy as np
import cv2
import imutils
from make_hogs import _process_image

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
		hog = json.load(file(hf))
		ds.append(hog)
		ls.append(l)
	ds = np.array(ds)
	return {'data': ds, 'labels': ls}

if __name__ == '__main__':


	IMG_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-02-18')
	DST_PATH = os.path.expanduser('output/images/predict')

	hdata = get_hog_data()

	(trainData, testData, trainLabels, testLabels) = train_test_split(hdata['data'], hdata['labels'], test_size=0.25, random_state=42)

	(trainData, valData, trainLabels, valLabels) = train_test_split(trainData, trainLabels, test_size=0.1, random_state=84)

	print valLabels

	# initialize the values of k for our k-Nearest Neighbor classifier along with the
	# list of accuracies for each value of k
	kVals = range(1, 30, 2)
	accuracies = []

	# loop over various values of `k` for the k-Nearest Neighbor classifier
	for k in xrange(1, 30, 2):
		# train the k-Nearest Neighbor classifier with the current value of `k`
		model = KNeighborsClassifier(n_neighbors=k)
		model.fit(trainData, trainLabels)

		# evaluate the model and update the accuracies list
		score = model.score(valData, valLabels)
		print("k=%d, accuracy=%.2f%%" % (k, score * 100))
		accuracies.append(score)

	# find the value of k that has the largest accuracy
	i = np.argmax(accuracies)
	print("k=%d achieved highest accuracy of %.2f%% on validation data" % (kVals[i], accuracies[i] * 100))

	# re-train our classifier using the best k value and predict the labels of the
	# test data
	model = KNeighborsClassifier(n_neighbors=kVals[i])
	model.fit(trainData, trainLabels)
	mfname = 'models/knc.pkl'
	joblib.dump(model, mfname)

	model = joblib.load(mfname)

#	predictions = model.predict(testData)

#	print predictions

	# show a final classification report demonstrating the accuracy of the classifier
	# for each of the digits
#	print("EVALUATION ON TESTING DATA")
#	print(classification_report(testLabels, predictions))

# sort real data
	for fn in glob.glob(os.path.join(IMG_PATH, '*.jpg')):
		image = cv2.imread(fn)
		hog = _process_image(image)
		prediction = model.predict([hog])
		dname = os.path.join(DST_PATH, os.path.basename(IMG_PATH), prediction[0])
		if not os.path.exists(dname):
			os.makedirs(dname)
		fname = os.path.join(dname, os.path.basename(fn))
		cv2.imwrite(fname, imutils.resize(image, width=160))
