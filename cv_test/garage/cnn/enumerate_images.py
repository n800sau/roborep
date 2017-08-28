#!/usr/bin/env python

import sys, os
from keras.preprocessing.image import ImageDataGenerator

SRC_DIR = 'input'
IMG_WIDTH = 224
IMG_HEIGHT = 224
BATCH_SIZE = 32

def image_reader(imgdir):
	pass


train_datagen = ImageDataGenerator(rescale=1./255)

train_generator = train_datagen.flow_from_directory(SRC_DIR,
		target_size=(IMG_WIDTH, IMG_HEIGHT),
		batch_size=BATCH_SIZE,
		class_mode='binary')

for d in train_generator:
	print d[0].shape, d[0].dtype, d[0].max(), d[0].min(), d[1]
	break

