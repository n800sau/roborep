#!/usr/bin/env python

import os, sys
import cv2
import numpy as np
from imutils import paths, resize

INPUT = 'input'
OUTPUT = 'output'

avg = None

for ifname in paths.list_images(INPUT):
	print ifname
	image = cv2.imread(ifname)
#	resized = resize(image, width=360)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	if avg is None:
		print("[INFO] starting background model...")
		avg = gray.copy().astype("float")
	else:
		# accumulate the weighted average between the current frame and
		cv2.accumulateWeighted(gray, avg, 0.5)

if not avg is None:
	print 'shape=', avg.shape
	np.save('avg', avg)
	cv2.imwrite(os.path.join(OUTPUT, 'avg.jpg'), cv2.convertScaleAbs(avg))
	print 'Finished'
else:
	print 'Nothing to calculate'
	sys.exit(5)
