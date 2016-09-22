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
from keras.layers import Activation, Dropout, Flatten, Dense
from keras.utils.visualize_util import plot

DO_TRAIN = 0


# dimensions of our images.
img_width, img_height = 150, 150

prefix = 'robo_'
train_data_dir = 'data/train'
validation_data_dir = 'data/validation'
nb_train_samples = 2000
nb_validation_samples = 800
nb_epoch = 1
weights_fname = prefix + 'weights.h5'

model = Sequential()



model.add(Convolution2D(32, 3, 3, input_shape=(3, img_width, img_height)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Convolution2D(32, 3, 3))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Convolution2D(64, 3, 3))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Flatten())
model.add(Dense(64))
model.add(Activation('relu'))
model.add(Dropout(0.5))
model.add(Dense(1))
model.add(Activation('sigmoid'))

model.compile(loss='binary_crossentropy',
              optimizer='rmsprop',
              metrics=['accuracy'])

if os.path.exists(weights_fname):
	model.load_weights(weights_fname)

# this is the augmentation configuration we will use for testing:
# only rescaling
test_datagen = ImageDataGenerator(rescale=1./255)

validation_generator = test_datagen.flow_from_directory(
        validation_data_dir,
        target_size=(img_width, img_height),
        batch_size=32,
        class_mode='binary')

print 'validate=', validation_generator.class_indices

if DO_TRAIN:

	# this is the augmentation configuration we will use for training
	train_datagen = ImageDataGenerator(
        rescale=1./255,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True)


	train_generator = train_datagen.flow_from_directory(
        train_data_dir,
        target_size=(img_width, img_height),
        batch_size=32,
        class_mode='binary')

	print 'train=', validation_generator.class_indices
	json.dump(validation_generator.class_indices, open(prefix + 'labels.json', "w"), indent=2)

	rs = model.fit_generator(
        train_generator,
        samples_per_epoch=nb_train_samples,
        nb_epoch=nb_epoch,
        validation_data=validation_generator,
        nb_val_samples=nb_validation_samples)

	print rs.epoch

	open(prefix + 'model.json', "w").write(model.to_json(indent=2))

	if os.path.exists(weights_fname):
		os.unlink(weights_fname)
	model.save_weights(weights_fname)

#	plot(model, to_file='model.png')

print [d.shape for d in validation_generator.next()]

(loss, accuracy) = model.evaluate_generator(validation_generator, nb_validation_samples)
print("[INFO] accuracy: {:.2f}%".format(accuracy * 100))

print 'Finished'
