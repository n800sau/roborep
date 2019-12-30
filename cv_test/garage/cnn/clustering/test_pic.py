#!/usr/bin/env python3

import os, datetime, pickle, time, glob
import numpy as np
from keras.preprocessing import image
from keras.applications.vgg16 import VGG16
from keras.applications.vgg16 import preprocess_input

KFNAME = 'output/clustered/kmeans.pickle'

TESTDIR = 'test_images'
PLOTDIR = 'output/plots'

clt = pickle.load(open(KFNAME, 'rb'))
model = VGG16(weights='imagenet', include_top=False)
model.summary()

result = {}
for fname in glob.glob(os.path.join(TESTDIR, '*_pics', '*.jpg')):
	img = image.load_img(fname, target_size=(224, 224))
	img_data = image.img_to_array(img)
	img_data = np.expand_dims(img_data, axis=0)
	img_data = preprocess_input(img_data)
	vgg16_feature = model.predict(img_data)
	labels = clt.predict(np.array([vgg16_feature.flatten()]))
	result[os.path.basename(fname)] = labels[0]

plotdata = {}
for k,v in result.items():
	f_id = k.split('-')[0]
	# 20180523165526-09-00...
	ts = datetime.datetime.strptime(f_id, '%Y%m%d%H%M%S')
	dt = ts.date().strftime('%Y%m%d')
	plotdata[dt] = plotdata.get(dt, [])
	plotdata[dt].append({
		'ts': ts,
		'v': v,
	})

pickle.dump(plotdata, open(os.path.join(PLOTDIR, 'plotdata.pickle'), 'wb'), pickle.HIGHEST_PROTOCOL)
