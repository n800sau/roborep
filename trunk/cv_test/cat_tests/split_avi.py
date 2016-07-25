import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import cv2, random, json, time
from imutils import paths, resize
import argparse
import numpy as np
from pyimagesearch.utils import Conf

from keras.models import model_from_json

from image_data import extract_image_data

# construct the argument parser and parse the command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--conf", required=True, help="path to configuration file")
ap.add_argument("-v", "--video", required=False, help="path to video file")
args = vars(ap.parse_args())

# load the configuration and grab all image paths in the dataset
conf = Conf(args["conf"])

# if a video path was not supplied, grab the reference to the webcam
if args.get("video", False):
	camera = cv2.VideoCapture(args["video"])
else:
	camera = cv2.VideoCapture(0)

outlist = {}
fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
fps = 25

icount = 1

hdata = None

model1 = model_from_json(open(conf["arch"], "r").read())
model1.load_weights(conf["weights"])

label_list1 = json.load(file(conf["labels"], "r"))

t = time.time()

try:
	i = 0
	while True:

		(grabbed, frame) = camera.read()

		# if we are viewing a video and we did not grab a frame, then we have reached the
		# end of the video
		if args.get("video") and not grabbed:
			break

		if not grabbed:
			continue

		data = extract_image_data(frame)

		if hdata is None:
			hdata = np.empty((icount, 1, data.shape[0], data.shape[1]), np.uint8)

		i += 1

		hdata[0][0] = data

		proba = model1.predict(hdata, batch_size=1, verbose=0)

#		print('PROBA:', proba)
		predictions = proba.argmax(axis=1)

		predict_label = label_list1[predictions[0]]

		print predict_label

		if predict_label not in outlist:
			sz = list(reversed(frame.shape[:2]))
			outlist[predict_label] = cv2.VideoWriter(predict_label + '.avi', fourcc, fps, tuple(sz))

		outlist[predict_label].write(frame)

finally:
	for out in outlist.values():
		out.release()
	print 'Finished (%d frame/sec)' % (i / (time.time() - t))
