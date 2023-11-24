# import the necessary packages
import numpy as np
import cv2
import h5py

def prepare_image(image, fixedSize):
	# convert the image from BGR to RGB, then resize it to a fixed size,
	# ignoring aspect ratio
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	image = cv2.resize(image, tuple(fixedSize))

	# return the image
	return image

def build_batch(paths, fixedSize):
	# load the images from disk, prepare them for extraction, and convert
	# the list to a NumPy array
	images = [prepare_image(cv2.imread(p), fixedSize) for p in paths]
	images = np.array(images, dtype="float")

	# extract the labels from the image paths
	labels = [":".join(p.split("/")[-2:]) for p in paths]

	# return the labels and images
	return (labels, images)

def chunk(l, n):
	# loop over the list `l`, yielding chunks of size `n`
	for i in np.arange(0, len(l), n):
		yield l[i:i + n]

def dump_dataset(data, labels, path, datasetName, writeMethod="w"):
	# open the database, create the dataset, write the data and labels to dataset,
	# and then close the database
	db = h5py.File(path, writeMethod)
	dataset = db.create_dataset(datasetName, (len(data), len(data[0]) + 1), dtype="float")
	dataset[0:len(data)] = np.c_[labels, data]
	db.close()

def load_dataset(path, datasetName):
	# open the database, grab the labels and data, then close the dataset
	db = h5py.File(path, "r")
	(labels, data) = (db[datasetName][:, 0], db[datasetName][:, 1:])
	db.close()

	# return a tuple of the data and labels
	return (data, labels)
