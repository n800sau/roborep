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
args = vars(ap.parse_args())

# load the configuration and grab all image paths in the dataset
conf = Conf(args["conf"])

fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
fps = 2

camera = cv2.VideoCapture(0)
camera.set(cv2.cv.CV_CAP_PROP_FPS, fps)
camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)

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

		print '%5d %s %s' % (i, predict_label, proba)

finally:
	print 'Finished (%d frame/sec)' % (i / (time.time() - t))
