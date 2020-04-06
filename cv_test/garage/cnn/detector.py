#!/usr/bin/env python

import sys, os

import cv2, json, time
import numpy as np
from keras.models import model_from_json
from keras.preprocessing import image

class Detector:

	# dimensions of our images.
	img_width, img_height = 150, 150
	weights_fname = 'weights.h5'

	def __init__(self):

		self.model = model_from_json(file('model.json').read())
		self.labels = dict([(v,k) for k,v in json.load(file('labels.json')).items()])
		self.model.load_weights(self.weights_fname)
		self.model.compile(loss='binary_crossentropy', optimizer='rmsprop', metrics=['accuracy'])


	def detect_label(self, imagedata):

		data = cv2.resize(imagedata, (self.img_width, self.img_height))

		data = np.swapaxes(data, 0, 2)

		print 'data shape', data.shape


		hdata = np.expand_dims(data/255.0, axis=0)


		t1 = time.time()
		proba = self.model.predict_proba(hdata)
#				proba = model.predict_classes(hdata)

		print('%.2f sec: PROBA:', time.time() - t1, proba)
#				predictions = proba.argmax(axis=1)

#				predict_label = labels[predictions[0]]
		if proba[0][0] > 0.6 or proba[0][0] < 0.4:
			predict_label = self.labels[proba[0][0] >= 0.5]
		else:
			predict_label = '-'

		return predict_label
