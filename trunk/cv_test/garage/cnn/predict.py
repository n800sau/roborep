#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import cv2, random, json, time, glob, imghdr
from imutils import paths, resize
import numpy as np
from keras.models import model_from_json
from keras.preprocessing import image
import imutils

#IMG_PATH = os.path.expanduser('~/work/garage/2016-10-02')
IMG_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-10-25')
#IMG_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-10-02')
#IMG_PATH = os.path.expanduser('data/train/inside')
DST_PATH = os.path.expanduser('output/images/predict')

weights_fname = 'weights.h5'

# dimensions of our images.
img_width, img_height = 150, 150

hdata = None

model = model_from_json(file('model.json').read())
labels = dict([(v,k) for k,v in json.load(file('labels.json')).items()])
model.load_weights(weights_fname)
model.compile(loss='binary_crossentropy',
              optimizer='rmsprop',
              metrics=['accuracy'])

t = time.time()

try:
	i = 0
	for root, dirs, files in os.walk(IMG_PATH, topdown=False, followlinks=True):
		for name in files:
			fn = os.path.join(root, name)
			if imghdr.what(fn) in ('jpeg', 'png'):
				img = image.load_img(fn, target_size=(img_width, img_height))

				imgdata = image.img_to_array(img)
				hdata = np.expand_dims(imgdata/255.0, axis=0)


#				data = cv2.resize(imagedata, (img_width, img_height))

#				hdata = np.empty((1, data.shape[2], data.shape[0], data.shape[1]), np.uint8)

#				data = np.swapaxes(data, 0, 2)

#				hdata[0] = data/255.0

				t1 = time.time()
				proba = model.predict_proba(hdata)
#				proba = model.predict_classes(hdata)

				print('%.2f sec: PROBA:', time.time() - t1, proba)
#				predictions = proba.argmax(axis=1)

#				predict_label = labels[predictions[0]]
				if proba[0][0] > 0.6 or proba[0][0] < 0.4:
					predict_label = labels[proba[0][0] >= 0.5]
				else:
					predict_label = '-'

				print predict_label

#				imagedata = cv2.imread(fn)
				dname = os.path.join(DST_PATH, os.path.basename(IMG_PATH), predict_label)
				if not os.path.exists(dname):
					os.makedirs(dname)
				fname = os.path.join(dname, os.path.basename(fn))
				os.symlink(fn, fname)
#				cv2.imwrite(fname, imutils.resize(imagedata, width=160))

				i += 1


finally:
	print 'Finished (%d image/sec)' % (i / (time.time() - t))
