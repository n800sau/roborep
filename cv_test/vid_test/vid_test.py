#!/usr/bin/env python

import sys, os
import cv2
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.cluster import KMeans

#for f in dir(cv2):
#<->print f
#sys.exit()
ifname = os.path.expanduser('~/work/opencv/vid_test/FILE0014.MOV')

sz = (640, 360)

#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(ifname)

# Define the codec and create VideoWriter object
fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')

out = cv2.VideoWriter('output.avi', fourcc, 30, sz)

detector = cv2.FeatureDetector_create("Dense")
detector.setInt("initXyStep", 6)
extractor = cv2.DescriptorExtractor_create("SIFT")

colors = ((255,0,0), (0,255,0), (0,0,255))

i = 0
while cap.isOpened():
	ret, frame = cap.read()
	if ret==True:
		# same output video and frame size is important
		frame = cv2.resize(frame, sz)

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		kps = detector.detect(gray)
		(kps, descs) = extractor.compute(gray, kps)
		if i == 0:
			model = KNeighborsClassifier(n_neighbors=1)
			clt = KMeans(n_clusters=3)
			clt.fit(descs)
			model.fit(descs, clt.labels_)
		else:
			predictions = model.predict(descs)
			groups = {}
			for i in range(len(predictions)):
				l = predictions[i]
				groups[l] = groups.get(l, {'kps': [], 'descs': []})
				groups[l]['kps'].append(kps[i])
				groups[l]['descs'].append(descs[i])

			for l,m in groups.items():
				frame = cv2.drawKeypoints(frame, m['kps'], color=colors[l])

		out.write(frame)
		i += 1
	else:
		print 'No more'
		break
else:
	print 'Can not open %s' % ifname

print 'processed %d frames' % i

# Release everything if job is finished
cap.release()
out.release()

