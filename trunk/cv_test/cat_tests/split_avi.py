import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import cv2, random, json, time
from imutils import paths, resize
import argparse
import numpy as np
from pyimagesearch.utils import Conf
from keras.optimizers import SGD
from keras.utils import np_utils
from pyimagesearch.cnn import ConvNetFactory

#from keras.layers.convolutional import Convolution2D
#from keras.layers.convolutional import MaxPooling2D
#from keras.layers.core import Activation
#from keras.layers.core import Flatten
#from keras.layers.core import Dropout
#from keras.layers.core import Dense
#from keras.models import Sequential
from keras.models import model_from_json

# construct the argument parser and parse the command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--conf", required=True, help="path to configuration file")
ap.add_argument("-v", "--video", required=True, help="path to video file")
args = vars(ap.parse_args())

# load the configuration and grab all image paths in the dataset
conf = Conf(args["conf"])

IMG_SIDE = 100

def extract_image_data(image):
	image = cv2.resize(image, (IMG_SIDE, IMG_SIDE))
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	h,s,v = cv2.split(hsv)
	hist = cv2.calcHist([h], [0], None, [256], [0, 256])
	hist /= hist.max()
	return hist.reshape(16, 16)


# if a video path was not supplied, grab the reference to the webcam
if args.get("video", False):
	camera = cv2.VideoCapture(args["video"])
else:
	camera = cv2.VideoCapture(0)

outlist = {}
fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
fps = 25

icount = 1

hdata = np.empty((icount, 1, 16, 16), np.uint8)

model1 = model_from_json(open(conf["arch"], "r").read())
model1.load_weights(conf["weights"])

label_list1 = json.load(file(conf["labels"], "r"))

t = time.time()
i = 0
while True:

	(grabbed, frame) = camera.read()

	# if we are viewing a video and we did not grab a frame, then we have reached the
	# end of the video
	if args.get("video") and not grabbed:
		break

	if not grabbed:
		continue

	i += 1

	hdata[0] = extract_image_data(frame)

	proba = model1.predict(hdata, batch_size=1, verbose=0)

#	print('PROBA:', proba)
	predictions = proba.argmax(axis=1)

	predict_label = label_list1[predictions[0]]

	print predict_label

	if predict_label not in outlist:
		sz = list(reversed(frame.shape[:2]))
		outlist[predict_label] = cv2.VideoWriter(predict_label + '.avi', fourcc, fps, tuple(sz))

	outlist[predict_label].write(frame)

for out in outlist.values():
	out.release()

print 'Finished (%d frame/sec)' % (i / (time.time() - t))
