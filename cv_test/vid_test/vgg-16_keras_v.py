#!/usr/bin/env python

import time, os, glob, imutils, sys
from imutils import paths
from keras.models import Sequential, model_from_json
from keras.layers.core import Flatten, Dense, Dropout
from keras.layers.convolutional import Convolution2D, MaxPooling2D, ZeroPadding2D
from keras.optimizers import SGD
import cv2, numpy as np

VID_PATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/n800s/bike/FILE0001.MOV')
MODEL_PATH = 'vgg-16_model.json'
WEIGHTS_PATH = '../../datasets/vgg16/vgg16_weights.h5'
SYNSETS_PATH = '../../datasets/vgg16/synsets.txt'
SYNSET_WORDS_PATH = '../../datasets/vgg16/synset_words.txt'
fps = 2

def dbprint(*args):
	for el in args:
		print el,
	print
	sys.stdout.flush()


def VGG_16(weights_path=None):

	if os.path.exists(MODEL_PATH):
		model = model_from_json(open(MODEL_PATH, "r").read())
	else:
		model = Sequential()
		model.add(ZeroPadding2D((1,1),input_shape=(3,224,224)))
		model.add(Convolution2D(64, 3, 3, activation='relu'))
		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(64, 3, 3, activation='relu'))
		model.add(MaxPooling2D((2,2), strides=(2,2)))

		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(128, 3, 3, activation='relu'))
		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(128, 3, 3, activation='relu'))
		model.add(MaxPooling2D((2,2), strides=(2,2)))

		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(256, 3, 3, activation='relu'))
		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(256, 3, 3, activation='relu'))
		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(256, 3, 3, activation='relu'))
		model.add(MaxPooling2D((2,2), strides=(2,2)))

		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(512, 3, 3, activation='relu'))
		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(512, 3, 3, activation='relu'))
		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(512, 3, 3, activation='relu'))
		model.add(MaxPooling2D((2,2), strides=(2,2)))

		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(512, 3, 3, activation='relu'))
		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(512, 3, 3, activation='relu'))
		model.add(ZeroPadding2D((1,1)))
		model.add(Convolution2D(512, 3, 3, activation='relu'))
		model.add(MaxPooling2D((2,2), strides=(2,2)))

		model.add(Flatten())
		model.add(Dense(4096, activation='relu'))
		model.add(Dropout(0.5))
		model.add(Dense(4096, activation='relu'))
		model.add(Dropout(0.5))
		model.add(Dense(1000, activation='softmax'))

		open(MODEL_PATH, "w").write(model.to_json(indent=2))

	if weights_path:
		model.load_weights(weights_path)

	return model

if __name__ == "__main__":

	words = file(SYNSETS_PATH).read().split('\n')
	word_names = dict([(nl[0],''.join(nl[1:])) for nl in [l.split(' ') for l in file(SYNSET_WORDS_PATH).read().split('\n')]])

	t = time.time()
	# Test pretrained model
	model = VGG_16(WEIGHTS_PATH)
	sgd = SGD(lr=0.1, decay=1e-6, momentum=0.9, nesterov=True)
	model.compile(optimizer=sgd, loss='categorical_crossentropy')

	tdiff = int(time.time() - t)
	dbprint('Loaded in %s:%s' % (tdiff // 60, tdiff % 60))

	camera = cv2.VideoCapture(VID_PATH)
	camera.set(cv2.cv.CV_CAP_PROP_FPS, fps)
	camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
	camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
	i = 0
	while True:


		(grabbed, image) = camera.read()

		if not grabbed:
			continue

		t = time.time()

		im = cv2.resize(image, (224, 224))
		im = im.astype(np.float32)
		im[:,:,0] -= 103.939
		im[:,:,1] -= 116.779
		im[:,:,2] -= 123.68
		im = im.transpose((2,0,1))
		im = np.expand_dims(im, axis=0)

		out = model.predict(im)
		pos = np.argmax(out)
		aword = word_names[words[pos]]
		tdiff = int(time.time() - t)
		cv2.putText(image, aword, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
#		cv2.imwrite(os.path.join('vgg-16', os.path.basename(fname)), imutils.resize(image, width=360))
#		dbprint(os.path.basename(fname), aword, ('%s:%s' % (tdiff // 60, tdiff % 60)))
		dbprint(i, aword, ('%s:%s' % (tdiff // 60, tdiff % 60)))
		i += 1

	dbprint('Finished')
