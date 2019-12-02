'''This script goes along the blog post
"Building powerful image classification models using very little data"
from blog.keras.io.

It uses data that can be downloaded at:
https://www.kaggle.com/c/dogs-vs-cats/data

In our setup, we:
- created a data/ folder
- created train/ and validation/ subfolders inside data/
- created cats/ and dogs/ subfolders inside train/ and validation/
- put the cat pictures index 0-999 in data/train/cats
- put the cat pictures index 1000-1400 in data/validation/cats
- put the dogs pictures index 12500-13499 in data/train/dogs
- put the dog pictures index 13500-13900 in data/validation/dogs

So that we have 1000 training examples for each class, and 400 validation examples for each class.

In summary, this is our directory structure:
```
data/
    train/
        dogs/
            dog001.jpg
            dog002.jpg
            ...
        cats/
            cat001.jpg
            cat002.jpg
            ...
    validation/
        dogs/
            dog001.jpg
            dog002.jpg
            ...
        cats/
            cat001.jpg
            cat002.jpg
            ...
```
'''

import os, json
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense, Input
from keras.optimizers import SGD
from keras.utils import plot_model
from keras.callbacks import ModelCheckpoint, ReduceLROnPlateau, TensorBoard, EarlyStopping

from model import build_model, IMG_TARGET_ROWS, IMG_TARGET_COLS
#from big_model import build_model

DO_TRAIN = 0
DO_PREDICT = 1

BATCH_SIZE = 16

# dimensions of our images.
IMG_WIDTH, IMG_HEIGHT = IMG_TARGET_COLS, IMG_TARGET_ROWS

train_data_dir = 'data/train'
validation_data_dir = 'data/validation'
steps_per_epoch = 128
validation_steps = 12
epochs = 300
weights_fname = 'weights.h5'

model = build_model()

open('model.json', "w").write(model.to_json(indent=2))


if os.path.exists(weights_fname):
	model.load_weights(weights_fname)

# this is the augmentation configuration we will use for testing:
# only rescaling
test_datagen = ImageDataGenerator(rescale=1./255)

validation_generator = test_datagen.flow_from_directory(
        validation_data_dir,
        target_size=(IMG_WIDTH, IMG_HEIGHT),
        batch_size=BATCH_SIZE,
        class_mode='binary')
#        class_mode='categorical')

if DO_TRAIN:

	# this is the augmentation configuration we will use for training
	train_datagen = ImageDataGenerator(rescale=1./255)

	train_generator = train_datagen.flow_from_directory(
        train_data_dir,
        target_size=(IMG_WIDTH, IMG_HEIGHT),
        batch_size=BATCH_SIZE,
        class_mode='binary')
#        class_mode='categorical')

	print 'train classes=', train_generator.class_indices
	json.dump(train_generator.class_indices, open('labels.json', "w"), indent=2)

#	rs = model.fit_generator(
#        train_generator,
#        samples_per_epoch=steps_per_epoch,
#        epochs=epochs,
#        nb_worker = 5,
#        validation_data=validation_generator,
#        nb_val_samples=validation_steps)

	reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.25, patience=4, min_lr=1e-6, verbose=1)

#	early_stopping = EarlyStopping(monitor='val_loss', patience=2)

	checkpoint = ModelCheckpoint('weights.{epoch:02d}-{val_loss:.2f}.hdf5', monitor='val_loss', verbose=0, save_best_only=True, save_weights_only=False, mode='auto', period=1)

	# API 2
	rs = model.fit_generator(train_generator, validation_data=validation_generator, validation_steps=validation_steps,
                        steps_per_epoch=steps_per_epoch, epochs=epochs, verbose=2,
                        callbacks=[reduce_lr, checkpoint], max_q_size=10)


#	print rs.epoch


	if os.path.exists(weights_fname):
		os.unlink(weights_fname)
	model.save_weights(weights_fname)

#plot_model(model, to_file='model.png')

labels = dict([(v,k) for k,v in json.load(file('labels.json')).items()])

#print [d.shape for d in validation_generator.next()]

print 'validation classes=', validation_generator.class_indices
print 'labels=', labels

(loss, accuracy) = model.evaluate_generator(validation_generator, validation_steps)
print("[INFO] accuracy: {:.2f}%".format(accuracy * 100))

# this is the augmentation configuration we will use for training
train_datagen = ImageDataGenerator(rescale=1./255)

train_generator = train_datagen.flow_from_directory(
        train_data_dir,
        target_size=(IMG_WIDTH, IMG_HEIGHT),
        batch_size=BATCH_SIZE,
        class_mode='binary')

(loss, accuracy) = model.evaluate_generator(train_generator, validation_steps)
print("[INFO] accuracy: {:.2f}%".format(accuracy * 100))

validation_generator.reset()

if DO_PREDICT:

#print('SIZE:', len(list(validation_generator)))

	max_n = 100
	right = 0
	failed = 0
	for x_data,y_data in validation_generator:
		for i in range(len(x_data)-1):
			print x_data[i].shape
#			print 1, model.predict(x_data[i:i+1], verbose=0), y_data[i]
#			print 2, model.predict_proba(x_data[i:i+1], verbose=0), y_data[i]
			print 3, model.predict_classes(x_data[i:i+1], verbose=0), y_data[i]
#			continue
#			proba = model.predict_classes(x_data[i:i+1], verbose=0)
#			print 'PROBA', proba, y_data[i]
#			if y_data[i][proba[0]]:
#				right += 1
#				predict_label = labels[proba[0]]

#				print(predict_label)
#			else:
#				print('FAILED')
#				failed += 1
		max_n -= 1
		if max_n <= 0:
			break

#		print('CLASSES:', proba, ',', y_data[i])

#		proba = model.predict_proba(x_data[i:i+1])

#		print('PROBA:', proba)

#		predictions = proba.argmax(axis=1)

#		print('LABELS:', labels)
#		print('PREDICTIONS:', predictions)

#		predict_label = labels[predictions[0]]

#	print 'Failed %d from %d, accuracy: (%.2f)' % (failed, failed + right, right/float(failed + right))

print 'Finished.'
