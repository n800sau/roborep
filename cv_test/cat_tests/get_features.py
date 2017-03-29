#!/usr/bin/env python

#from keras.applications.vgg16 import VGG16
#from keras.applications.vgg16 import preprocess_input
#from keras.applications.vgg19 import VGG19
#from keras.applications.vgg19 import preprocess_input
#from keras.applications.xception import Xception
#from keras.applications.xception import preprocess_input
#from keras.applications.inception_v3 import InceptionV3
#from keras.applications.inception_v3 import preprocess_input
from keras.applications.resnet50 import ResNet50
from keras.applications.resnet50 import preprocess_input

from keras.preprocessing import image
from keras.models import Model
import numpy as np
from scipy.misc import imsave
from glob import glob
import os, sys, imghdr, time

#SRCDIR = os.path.expanduser('~/work/garage/2016-09-07')
SRCDIR = os.path.expanduser('~/work/garage/2016-11-02')
#SRCDIR = os.path.expanduser('~/work/garage/random')
OUTDIR = 'output'

if not os.path.exists(OUTDIR):
	os.makedirs(OUTDIR)

IMGDIM = (224, 224) # vgg
#IMGDIM = (299, 299) # insertion, xsertion

#OIMGDIM = (16, 32) #vgg
#OIMGDIM = (2, 5) # xcertion
OIMGDIM = (64, 32) #insertion v3, resnet50

#model = VGG16(weights='imagenet', include_top=False)

#base_model = VGG19(weights='imagenet')
#model = Model(input=base_model.input, output=base_model.get_layer('block4_pool').output)

#model = Xception(include_top=False)
#base_model = Xception()
#model = Model(input=base_model.input, output=base_model.get_layer('block13_pool').output)

#model = InceptionV3(include_top=False)

model = ResNet50(include_top=False)

def get_features(img_path):

	img = image.load_img(img_path, target_size=IMGDIM)

	x = image.img_to_array(img)
	# for xseption
#	x = np.swapaxes(x, 0, 2)

	x = np.expand_dims(x, axis=0)
	x = preprocess_input(x)
	return model.predict(x)

i = 0
for root, dirs, files in os.walk(SRCDIR, followlinks=True):
	for fname in files:
		fname = os.path.join(os.path.join(root, fname))
		try:
			itype = imghdr.what(fname)
			if itype :
				t = time.time()
				print 'Get Features'
				features = get_features(fname)
				print '%3d (%.2f s): %s' % (i, time.time() - t, fname)
				features = features[0].mean(axis=(1,2))
				print features.shape
				np.save(os.path.join(OUTDIR, os.path.basename(fname)), features)
				features = features.reshape(OIMGDIM)
#				imsave(os.path.join(OUTDIR, os.path.basename(fname)), features)
				os.symlink(fname, os.path.join(OUTDIR, os.path.basename(fname)))
				i += 1
		except Exception, e:
			print e

print 'Finished'
