import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import cv2, random, json
from imutils import paths, resize
import argparse
import numpy as np
from pyimagesearch.utils import Conf
from keras.optimizers import SGD
from keras.utils import np_utils
from pyimagesearch.cnn import ConvNetFactory

from keras.layers.convolutional import Convolution2D
from keras.layers.convolutional import MaxPooling2D
from keras.layers.core import Activation
from keras.layers.core import Flatten
from keras.layers.core import Dropout
from keras.layers.core import Dense
from keras.models import Sequential

from image_data import extract_image_data

DO_TRAIN = 1
DO_TEST = 1

# construct the argument parser and parse the command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--conf", required=True, help="path to configuration file")
args = vars(ap.parse_args())

# load the configuration and grab all image paths in the dataset
conf = Conf(args["conf"])
imagePaths = list(paths.list_images(conf["dataset_path"]))
random.shuffle(imagePaths)
icount = len(imagePaths)

print 'Files:', icount

IMG_SIDE = 100

label_list = []
labels = np.empty(icount, np.uint8)
hdata = None 
idata = np.empty((icount, 3, IMG_SIDE, IMG_SIDE), np.uint8)

# collect labels
label_list = sorted(set([os.path.basename(os.path.dirname(fname)) for fname in imagePaths]))

i = 0
for fname in imagePaths:
	label = os.path.basename(os.path.dirname(fname))
	image = cv2.imread(fname)
	imgdata = extract_image_data(image)
	if hdata is None:
		hdata = np.empty((icount, 1, imgdata.shape[0], imgdata.shape[1]), np.uint8)
	hdata[i][0] = imgdata
#	image = cv2.imread(fname)
#	image = cv2.resize(image, (IMG_SIDE, IMG_SIDE))
# not working like that
#	image = resize(image, width=100, height=100)
#	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#	h,s,v = cv2.split(hsv)
#	hist = cv2.calcHist([h], [0], None, [256], [0, 256])
#	hist /= hist.max()
	label_index = label_list.index(label)
	labels[i] = label_index
#	hdata[i][0] = hist.reshape(16, 16)
#	idata[i] = np.swapaxes(cv2.resize(image, (IMG_SIDE, IMG_SIDE)), 0, 2)
	i += 1

data = hdata

idxs_split = int(len(data) * (1.0 - conf['test_size']))


(trainData, testData) = (data[:idxs_split], data[idxs_split:])
(trainLabelList, testLabelList) = (labels[:idxs_split], labels[idxs_split:])


# transform the training and testing labels into vectors in the range
# [0, numClasses] -- this generates a vector for each label, where the
# index of the label is set to `1` and all other entries to `0`; in the
# case of CIFAR-10, there are 10 class labels
trainLabels = np_utils.to_categorical(trainLabelList, 2)
testLabels = np_utils.to_categorical(testLabelList, 2)

#print 'Here1', testLabelList
#sys.stdout.flush()

#print 'Here2', testLabels
#sys.stdout.flush()


if DO_TRAIN:

	kargs = {"dropout": conf["dropout"] > 0, "activation": conf["activation"]}

	#model = ConvNetFactory.build(conf["network"], 1, 16, 16, 2, **kargs)
	#model = ConvNetFactory.build(conf["network"], 3, IMG_SIDE, IMG_SIDE, 2, **kargs)

	model = Sequential()

	print '###', data[0].shape

	# define the first set of CONV => ACTIVATION => POOL layers
	model.add(Convolution2D(32, 3, 3, border_mode="same", input_shape=data[0].shape))
	model.add(Activation(kargs['activation']))

	model.add(Convolution2D(32, 3, 3))
	model.add(Activation(kargs['activation']))
	model.add(MaxPooling2D(pool_size=(2, 2)))
	model.add(Dropout(0.25))

#	model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))


	# define the second set of CONV => ACTIVATION => POOL layers
#	model.add(Convolution2D(50, 5, 5, border_mode="same"))
	model.add(Convolution2D(64, 3, 3, border_mode="same"))
	model.add(Activation(kargs['activation']))
	model.add(Convolution2D(64, 3, 3))
	model.add(Activation(kargs['activation']))
	model.add(MaxPooling2D(pool_size=(2, 2)))
#	model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
	model.add(Dropout(0.25))

	# define the first FC => ACTIVATION layers
	model.add(Flatten())
	model.add(Dense(512))
	model.add(Activation(kargs['activation']))
	model.add(Dropout(0.25))

	# define the second FC layer
	model.add(Dense(2))

	# lastly, define the soft-max classifier
	model.add(Activation("softmax"))

	#model = ConvNetFactory.build(conf["network"])
	sgd = SGD(lr=0.01, decay=1e-6, momentum=0.9, nesterov=True)
	model.compile(loss="categorical_crossentropy", optimizer=sgd, metrics=["accuracy"])

	print("[INFO] starting training...")
	model.fit(trainData, trainLabels, batch_size=conf["batch_size"], nb_epoch=conf["epochs"], verbose=2)

	# show the accuracy on the testing set
	(loss, accuracy) = model.evaluate(testData, testLabels, batch_size=conf["batch_size"], verbose=2)
	#(loss, accuracy) = model.evaluate(testData, testLabels, batch_size=conf["batch_size"], metrics=["accuracy"], verbose=2)
	print("[INFO] accuracy: {:.2f}%".format(accuracy * 100))

	# dump the network architecture and weights to file
	print("[INFO] dumping architecture to file " + conf["arch"])
	open(conf["arch"], "w").write(model.to_json(indent=2))
	print("[INFO] dumping weight to file " + conf["weights"])
	model.save_weights(conf["weights"], overwrite=True)

	json.dump(label_list, file(conf["labels"], "w"), indent=2)


if DO_TEST:

	from keras.models import model_from_json
	model1 = model_from_json(open(conf["arch"], "r").read())
	model1.load_weights(conf["weights"])

	label_list1 = json.load(file(conf["labels"], "r"))

	proba = model1.predict(testData, batch_size=conf["batch_size"], verbose=1)
	print('PROBA:', proba)
	predictions = proba.argmax(axis=1)

	right = 0
	for (i, prediction) in enumerate(predictions):
		label = label_list1[testLabelList[i]]
		predict_label = label_list1[prediction]
		if testLabelList[i] == prediction:
			right += 1
		print("[INFO] predicted: {}, actual: {}".format(predict_label, label))

	print('Accuracy: %d%%' % (float(right) / i * 100))

print 'Finished'
