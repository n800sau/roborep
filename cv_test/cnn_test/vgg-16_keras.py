#!/usr/bin/env python

import time, os, glob, imutils, sys
from imutils import paths
from keras.models import Sequential, model_from_json
from keras.layers.core import Flatten, Dense, Dropout
from keras.layers.convolutional import Convolution2D, MaxPooling2D, ZeroPadding2D
from keras.optimizers import SGD
import cv2, numpy as np

#IMAGE_PATH = 'dogs_and_cats/cat'
IMAGE_PATH = 'pics'

MODEL_PATH = 'vgg-16_model.json'
WEIGHTS_PATH = '../../../datasets/vgg16/vgg16_weights.h5'
SYNSETS_PATH = '../../../datasets/vgg16/synsets.txt'
SYNSET_WORDS_PATH = '../../../datasets/vgg16/synset_words.txt'
OUTPUT_PATH = 'vgg-16'

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

def pyramid(image, scale=1.5, minSize=(224, 224)):
	# yield the original image
	yield image

	# keep looping over the pyramid
	while True:
		# compute the new dimensions of the image and resize it
		w = int(image.shape[1] / scale)
		print 'w=', w
		image = imutils.resize(image, width=w)

		# if the resized image does not meet the supplied minimum
		# size, then stop constructing the pyramid
		if image.shape[0] < minSize[1] or image.shape[1] < minSize[0]:
			break

		# yield the next image in the pyramid
		yield image

def sliding_window(image, stepSize=112, windowSize=(224, 224)):
	# slide a window across the image
	for y in xrange(0, image.shape[0], stepSize):
		for x in xrange(0, image.shape[1], stepSize):
			# yield the current window
			window = image[y:y + windowSize[1], x:x + windowSize[0]]
			# if the current window does not meed our desired window size, ignore it
			if window.shape[0] != windowSize[1] or window.shape[1] != windowSize[0]:
				continue
#			dbprint(x,y,window.shape)
			yield (x, y, window)


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

	i = 0
	for fname in paths.list_files(IMAGE_PATH):

		t = time.time()
		image = cv2.imread(fname)
		sq_image = cv2.resize(image, (224, 224) )
#		sq_image = cv2.resize(image, (224*2, 224*2) )

		wlist = set()
		window = sq_image
		# loop over the image pyramid
#		for layer in pyramid(sq_image, scale=2):
			# loop over the sliding window for each layer of the pyramid
#			for (x, y, window) in sliding_window(layer, windowSize=(224, 224)):

#				window = cv2.resize(window, (224, 224))
		im = window.astype(np.float32)
		im[:,:,0] -= 103.939
		im[:,:,1] -= 116.779
		im[:,:,2] -= 123.68
		im = im.transpose((2,0,1))
		im = np.expand_dims(im, axis=0)

		out = model.predict(im)
		pos = np.argmax(out)
		out_idx = np.argsort(out)[0,-5:]
		aword = word_names[words[pos]]
		dbprint(os.path.basename(fname), aword)
		for idx in reversed(out_idx):
			dbprint(word_names[words[idx]], out[0, idx])
		wlist.add(aword)

		w = ';'.join(list(wlist))
		tdiff = int(time.time() - t)
		image = imutils.resize(image, width=360)
		cv2.putText(image, w, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
		dpath = os.path.join(OUTPUT_PATH, w)
		if not os.path.exists(dpath):
			os.makedirs(dpath) 
		cv2.imwrite(os.path.join(dpath, os.path.basename(fname)), image)
		dbprint(os.path.basename(fname), w, ('%s:%s' % (tdiff // 60, tdiff % 60)))
		i += 1
		if i > 5:
			break

	dbprint('Finished')
