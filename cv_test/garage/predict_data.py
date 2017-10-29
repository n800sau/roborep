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

if __name__ == '__main__':


	IMG_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-11-25')
	DST_PATH = os.path.expanduser('output/images/predict')

	mfname = 'models/inside_empty.svc'
	model = joblib.load(mfname)

# sort real data
	for fn in glob.glob(os.path.join(IMG_PATH, '*.jpg')):
		image = cv2.imread(fn)
		hog = _process_image(image, mask_proc=car_mask)
		prediction = model.predict([hog])
		dname = os.path.join(DST_PATH, os.path.basename(IMG_PATH), prediction[0])
		if not os.path.exists(dname):
			os.makedirs(dname)
		fname = os.path.join(dname, os.path.basename(fn))
		cv2.imwrite(fname, imutils.resize(image, width=160))
