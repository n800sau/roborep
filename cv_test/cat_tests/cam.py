import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib'))

import cv2, random, json, time
from imutils import paths, resize
import redis
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

REDIS_KEY = 'LCD_timed_lines'
r = redis.Redis()

fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
fps = 2

camera = cv2.VideoCapture(0)
camera.set(cv2.cv.CV_CAP_PROP_FPS, fps)
camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)

icount = 1

hdata = None

model1 = model_from_json(open(conf["arch"], "r").read())
model1.compile(loss='binary_crossentropy',
              optimizer='rmsprop',
              metrics=['accuracy'])
model1.load_weights(conf["weights"])

labels = json.load(file(conf["labels"], "r"))

print 'Start processing'

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

		proba = model1.predict_classes(hdata, batch_size=1, verbose=0)

#		print('PROBA:', proba)
		predict_label = labels[proba[0]]

		print '%5d %s' % (i, predict_label)
		r.set(REDIS_KEY, time.strftime('%H:%M:%S') + '\n' + predict_label)
		r.expire(REDIS_KEY, 5)

finally:
	print 'Finished (%d frame/sec)' % (i / (time.time() - t))
