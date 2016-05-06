#!/usr/bin/env python

from __future__ import print_function
import os, dlib, json, cv2, glob
from imutils import paths
from skimage import io

BASEPATH = 'data'

print("[INFO] gathering images and bounding boxes...")
options = dlib.simple_object_detector_training_options()
options.be_verbose =True
#options.num_threads = 4
options.epsilon = 0.01
options.detection_window_size = 1600
print('size: %s' % options.detection_window_size)

images = []
boxes = []
for imPath in paths.list_images(os.path.join(BASEPATH, 'images')):

	bname = os.path.basename(imPath)
	dPath = os.path.join(BASEPATH, 'drawers', bname + '.json')
	if os.path.exists(dPath):
		images.append(io.imread(imPath))
		bb = [dlib.rectangle(left=long(b['x1']), top=long(b['y1']), right=long(b['x2']), bottom=long(b['y2'])) for b in json.load(file(dPath))]
		boxes.append(bb)

print("[INFO] training detector...")
detector = dlib.train_simple_object_detector(images, boxes, options)
print('detector:%s' % detector)
print("[INFO] dumping classifier to file...")
detector.save(os.path.join(BASEPATH, 'detector', 'simple.svm'))

print("[INFO] finished")
