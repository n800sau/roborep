#!/usr/bin/env python

from keras.applications.vgg16 import VGG16
from keras.preprocessing import image
from keras.applications.vgg16 import preprocess_input
import numpy as np
from scipy.misc import imsave
from glob import glob
import os, sys, imghdr, time

SRCDIR = os.path.expanduser('~/work/garage/2016-09-07')
OUTDIR = 'output'

model = VGG16(weights='imagenet', include_top=False)

def get_features(img_path):

	img = image.load_img(img_path, target_size=(224, 224))

	x = image.img_to_array(img)
	x = np.expand_dims(x, axis=0)
	x = preprocess_input(x)
	return model.predict(x)

i = 0
for root, dirs, files in os.walk(SRCDIR, followlinks=True):
	for fname in files:
		fname = os.path.join(os.path.join(root, fname))
		try:
			t = time.time()
			itype = imghdr.what(fname)
			if itype :
				features = get_features(fname)
				features = features[0].mean(axis=(1,2))
				print features.shape
				np.save(os.path.join(OUTDIR, os.path.basename(fname)), features)
				features = features.reshape((16, 32))
#				imsave(os.path.join(OUTDIR, os.path.basename(fname)), features)
				i += 1
				print '%3d (%.2f s): %s' % (i, time.time() - t, fname)
		except Exception, e:
			print e

print 'Finished'
