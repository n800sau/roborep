#!/usr/bin/env python

from __future__ import print_function
import os, dlib, json, cv2, glob
from imutils import paths
from skimage import io

print("[INFO] gathering images and bounding boxes...")
options = dlib.simple_object_detector_training_options()
options.be_verbose =True
#options.num_threads = 4
options.epsilon = 0.01
options.detection_window_size = 1600
print('size: %s' % options.detection_window_size)

images = []
boxes = []
for imPath in paths.list_images('images'):

	bname = os.path.basename(imPath)
	dPath = os.path.join('drawers', bname + '.json')
	if os.path.exists(dPath):
		images.append(io.imread(imPath))
		bb = [dlib.rectangle(left=long(b['x1']), top=long(b['y1']), right=long(b['x2']), bottom=long(b['y2'])) for b in json.load(file(dPath))]
		boxes.append(bb)

print("[INFO] training detector...")
detector = dlib.train_simple_object_detector(images, boxes, options)
print('detector:%s' % detector)
print("[INFO] dumping classifier to file...")
detector.save('detector.svm')

fnames = []

for dname in glob.glob(os.path.join('testing', '2016-*-*')):
	if os.path.isdir(dname):

		bdname = os.path.basename(dname)

		# loop over the testing images
		for testingPath in paths.list_images(dname):
			# load the image and make predictions
			image = cv2.imread(testingPath)
			boxes = detector(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

			outpath = os.path.join('output', bdname, 'no' if len(boxes) == 0 else 'yes')

			# loop over the bounding boxes and draw them
			for b in boxes:
				(x, y, w, h) = (b.left(), b.top(), b.right(), b.bottom())
				print(x, y, w, h)
				cv2.rectangle(image, (x, y), (w, h), (0, 255, 0), 2)

			# show the image
			if not os.path.exists(outpath):
				os.makedirs(outpath)
			ofname = os.path.join(outpath, os.path.basename(testingPath))
			cv2.imwrite(ofname, image)

print("[INFO] finished")
