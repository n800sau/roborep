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
options.add_left_right_image_flips = True;
options.epsilon = 0.01
options.detection_window_size = 1600
print('size: %s' % options.detection_window_size)

max_n = 100
aratio = None
images = []
boxes = []
skipped = 0
for imPath in paths.list_images(os.path.join(BASEPATH, 'images')):

	bname = os.path.basename(imPath)
	dPath = os.path.join(BASEPATH, 'drawers', bname + '.json')
	if os.path.exists(dPath):
		print('%s' % bname)
		bb = []
		for b in json.load(file(dPath)):
			b = dlib.rectangle(left=long(b['x1']), top=long(b['y1']), right=long(b['x2']), bottom=long(b['y2']))
			b_aratio = float(b.width())/b.height()
			print('size:%dx%d aspect:%.2f area:%d' % (b.width(), b.height(), b_aratio, b.area()))
			if aratio is None:
				aratio = b_aratio
			else:
				if (aratio >= 1 and b_aratio < 1) or (aratio < 1 and b_aratio >= 1):
					b = None
			if not b is None:
				bb.append(b)
		if bb:
			images.append(io.imread(imPath))
			boxes.append(bb)
			if len(images) >= max_n:
				break
		else:
			skipped += 1

print("[INFO] Using %d images (%d boxes) to train. Skipped %d images" % (len(images), len(boxes), skipped))

print("[INFO] training detector...")
detector = dlib.train_simple_object_detector(images, boxes, options)
print("[INFO] dumping classifier to file...")
detector.save(os.path.join(BASEPATH, 'detector', 'simple.svm'))

print("[INFO] finished")
