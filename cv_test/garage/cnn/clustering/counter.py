#!/usr/bin/env python3
import glob, os
import numpy as np
from keras.preprocessing import image
from keras.applications.vgg16 import VGG16
from keras.applications.vgg16 import preprocess_input

DIRPATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
NPDIR = 'output/npdata'

model = VGG16(weights='imagenet', include_top=False)
model.summary()

n = 0
for dname in glob.glob(os.path.join(DIRPATH, '*')):
	if os.path.isdir(dname):
		if dname.endswith('_pics'):
			arl = {}
			for fname in glob.glob(os.path.join(dname, '*.jpg')):
				print(fname)
				img = image.load_img(fname, target_size=(224, 224))
				img_data = image.img_to_array(img)
				img_data = np.expand_dims(img_data, axis=0)
				img_data = preprocess_input(img_data)
				vgg16_feature = model.predict(img_data)
				# vgg16_feature.shape - (1, 7, 7, 512)
				print(vgg16_feature.shape)
				bdfname = os.path.join(os.path.basename(os.path.dirname(fname)), os.path.basename(fname))
				arl[bdfname] = vgg16_feature
			if arl:
				dfname = os.path.join(NPDIR, os.path.basename(dname))
				if not os.path.exists(os.path.dirname(dfname)):
					os.makedirs(os.path.dirname(dfname))
				np.savez_compressed(dfname, **arl)
			n += 1
		if n > 3:
			break
