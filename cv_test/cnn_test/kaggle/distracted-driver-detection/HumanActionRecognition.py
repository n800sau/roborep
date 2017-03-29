'''This script reuses pieces of code from the post:
"Building powerful image classification models using very little data"
from blog.keras.io
and from:
https://www.kaggle.com/tnhabc/state-farm-distracted-driver-detection/keras-sample
The training data can be downloaded at:
https://www.kaggle.com/c/state-farm-distracted-driver-detection/data
'''

import os, sys, random, json, imghdr, time
import h5py
import numpy as np
from keras.preprocessing.image import ImageDataGenerator
from keras.preprocessing import image
from keras import optimizers
from keras.models import Sequential, Model
from keras.layers import Conv2D, MaxPooling2D, ZeroPadding2D
from keras.layers import Activation, Dropout, Flatten, Dense
from keras.callbacks import ModelCheckpoint, ReduceLROnPlateau, TensorBoard, EarlyStopping
from EigenvalueDecay import EigenvalueRegularizer
from keras.applications.vgg16 import VGG16
from keras.models import Model
from numpy.random import permutation
from keras.optimizers import SGD
import pandas as pd
import datetime
import glob
import cv2
import math
import pickle
from collections import OrderedDict
from keras import backend as K

DO_TRAIN = 0

SAVED_WEIGHTS = 'whole_model.h5'
OUTPUT = 'output'

# Enter here the path to the model weights files:
weights_path = 'vgg16_weights.h5'
# Enter here the path to the top-model weights files:
top_model_weights_path = 'fc_model.h5'
# Enter here the path for storage of the whole model weights (VGG16+top classifier model):
whole_model_weights_path = 'whole_model.h5'
# Enter here the name of the folder that contains the folders c0, c1,..., c9, with the training images belonging to classes 0 to 9:
train_data_dir = 'drivers/train'
# Enter here the name of the folder where is the test images (the data evalueted in the private leaderboard):
val_data_dir = 'drivers/train'

test_data_dir = 'drivers/test'

# Enter here the features of the data set:
img_width, img_height = 224, 224
steps_per_epoch = 100
val_steps = 200

#n_train_samples = 22424
#n_test_samples = 79726
n_test_samples = 1
color_type_global = 3

# You can set larger values here, according with the memory of your GPU:
batch_size = 164

# Enter here the number of training epochs (with 80 epochs the model was positioned among
# the 29% best competitors in the private leaderboard of state-farm-distracted-driver-detection)
# According to our results, this model can achieve a better performance if trained along a larger
# number of epochs, due to the agressive regularization with Eigenvalue Decay that was adopted.
nb_epoch = 1

# build the VGG16 network:
#model = Sequential()

#model.add(ZeroPadding2D((1, 1), input_shape=(img_width, img_height, 3)))
#model.add(Conv2D(64, (3, 3), activation='relu', name='conv1_1'))
#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(64, (3, 3), activation='relu', name='conv1_2'))
#model.add(MaxPooling2D((2, 2), strides=(2, 2)))

#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(128, (3, 3), activation='relu', name='conv2_1'))
#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(128, (3, 3), activation='relu', name='conv2_2'))
#model.add(MaxPooling2D((2, 2), strides=(2, 2)))

#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(256, (3, 3), activation='relu', name='conv3_1'))
#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(256, (3, 3), activation='relu', name='conv3_2'))
#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(256, (3, 3), activation='relu', name='conv3_3'))
#model.add(MaxPooling2D((2, 2), strides=(2, 2)))

#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(512, (3, 3), activation='relu', name='conv4_1'))
#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(512, (3, 3), activation='relu', name='conv4_2'))
#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(512, (3, 3), activation='relu', name='conv4_3'))
#model.add(MaxPooling2D((2, 2), strides=(2, 2)))

#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(512, (3, 3), activation='relu', name='conv5_1'))
#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(512, (3, 3), activation='relu', name='conv5_2'))
#model.add(ZeroPadding2D((1, 1)))
#model.add(Conv2D(512, (3, 3), activation='relu', name='conv5_3'))
#model.add(MaxPooling2D((2, 2), strides=(2, 2)))

base_model = VGG16(include_top=False, weights='imagenet', input_tensor=None, input_shape=(img_width, img_height, 3))


# loading the weights of the pre-trained VGG16:

#assert os.path.exists(weights_path), 'Model weights not found (see "weights_path" variable in script).'
#f = h5py.File(weights_path)
#for k in range(f.attrs['nb_layers']):
#	if k >= len(model.layers):
#		break
#	g = f['layer_{}'.format(k)]
#	weights = [g['param_{}'.format(p)] for p in range(g.attrs['nb_params'])]
#	model.layers[k].set_weights(weights)
#f.close()
#print('Model loaded.')

# building a classifier model on top of the convolutional model:

x = base_model.output
#top_model = Sequential()
#top_model.add(Flatten(input_shape=model.output_shape[1:]))
x = Flatten()(x)
#top_model.add(Dense(64, activation='relu', W_regularizer=EigenvalueRegularizer(10)))
#top_model.add(Dense(10, activation='softmax', W_regularizer=EigenvalueRegularizer(10)))
#top_model.add(Dense(64, activation='relu'))
x = Dense(64, activation='relu')(x)
#top_model.add(Dense(10, activation='softmax'))
predictions = Dense(10, activation='softmax')(x)
#top_model.load_weights(top_model_weights_path)

# add the model on top of the convolutional base
#model.add(top_model)

model = Model(inputs=base_model.input, outputs=predictions)

if os.path.exists(SAVED_WEIGHTS):
	model.load_weights(SAVED_WEIGHTS)
	print('Loaded weights {}'.format(SAVED_WEIGHTS))

# setting the first 15 layers to non-trainable (the original weights will not be updated)

#for layer in model.layers[:15]:
#	layer.trainable = False

for layer in base_model.layers[:15]:
	layer.trainable = False

# Compiling the model with a SGD/momentum optimizer:

model.compile(loss = "categorical_crossentropy",
		optimizer=optimizers.SGD(lr=1e-6, momentum=0.9),
		metrics=['mean_squared_logarithmic_error', 'accuracy'])

if DO_TRAIN:
	train_datagen = ImageDataGenerator(shear_range=0.3, zoom_range=0.3, rotation_range=0.3)

	print('trainning')
	train_generator = train_datagen.flow_from_directory(
		train_data_dir,
		target_size=(img_height, img_width),
		batch_size=batch_size,
		class_mode='categorical',
		shuffle=True)

	print('train class indices:', train_generator.class_indices)
	#print('train classes:', train_generator.classes, train_generator.class_indices)
	json.dump(train_generator.class_indices, open('labels.json', "w"), indent=2)

	class_dictionary = train_generator.class_indices

	val_datagen = ImageDataGenerator()
	val_generator = val_datagen.flow_from_directory(
		val_data_dir,
		target_size=(img_height, img_width),
		batch_size=batch_size,
		class_mode='categorical',
		shuffle=True)



else:
	class_dictionary = json.load(file('labels.json'))

classes = dict([(i, cl) for cl,i in class_dictionary.items()])

sorted_class_dictionary = OrderedDict(sorted(class_dictionary.items()))
print('Sorted classes:', sorted_class_dictionary)

if DO_TRAIN:

	reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.25, patience=4, min_lr=1e-6, verbose=1)

	checkpoint = ModelCheckpoint('weights.{epoch:02d}-{val_loss:.2f}.hdf5', monitor='val_loss', verbose=0, save_best_only=True, save_weights_only=False, mode='auto', period=1)

	# Fine-tuning the model:
	model.fit_generator(
		train_generator,
		steps_per_epoch=steps_per_epoch,
		epochs=nb_epoch,
		validation_data=val_generator,
		validation_steps=val_steps,
		callbacks=[reduce_lr, checkpoint],
		verbose=2)

	model.save_weights(whole_model_weights_path)



test_datagen = ImageDataGenerator()
test_generator = test_datagen.flow_from_directory(
	test_data_dir,
	target_size=(img_height, img_width),
	batch_size=batch_size,
	class_mode=None,
	shuffle=False)

aux = model.predict_generator(test_generator, n_test_samples)

clmap = json.load(file('classes.map'))

i = 0
for fn in test_generator.filenames[:aux.shape[0]]:
	cl = classes[np.argmax(aux[i])]
	srcfn = os.path.abspath(os.path.join(test_data_dir, fn))
	lname = os.path.join(OUTPUT, clmap[cl], os.path.basename(fn))
	if not os.path.exists(os.path.dirname(lname)):
		os.makedirs(os.path.dirname(lname))
	print fn, cl, clmap[cl]
	os.symlink(srcfn, lname)
	i += 1

print('Finished');


