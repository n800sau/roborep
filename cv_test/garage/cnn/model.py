from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense, Input
from keras.optimizers import SGD, Adam

IMG_TARGET_ROWS = 122
IMG_TARGET_COLS = 122
N_CLASSES = 1

# LeakyReLU
# PReLU
# ELU

def build_model():

	model = Sequential()

	model.add(Conv2D(32, (3, 3), padding='same', input_shape=(IMG_TARGET_ROWS, IMG_TARGET_COLS, 3)))
	model.add(Activation('relu'))

	model.add(Conv2D(32, (3, 3)))
	model.add(Activation('relu'))
	model.add(MaxPooling2D(pool_size=(2, 2)))
	model.add(Dropout(0.25))

	model.add(Conv2D(64, (3, 3), padding='same'))
	model.add(Activation('relu'))

	model.add(Conv2D(64, (3, 3)))
	model.add(Activation('relu'))
	model.add(MaxPooling2D(pool_size=(2, 2)))
	model.add(Dropout(0.25))

#	model.add(Conv2D(128, (3, 3), padding='same'))
#	model.add(Activation('relu'))

#	model.add(Conv2D(128, (3, 3)))
#	model.add(Activation('relu'))
#	model.add(MaxPooling2D(pool_size=(2, 2)))
#	model.add(Dropout(0.25))

	model.add(Flatten())
	model.add(Dense(512))
	model.add(Activation('relu'))
	model.add(Dropout(0.5))
	model.add(Dense(N_CLASSES))
#	model.add(Activation('relu'))
	model.add(Activation('softmax'))

	# Let's train the model using RMSprop
#	model.compile(loss='categorical_crossentropy', optimizer='rmsprop', metrics=['accuracy'])
	model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

	return model

if __name__ == '__main__':
	model = build_model()
	model.summary()

