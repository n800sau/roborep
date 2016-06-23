#!/usr/bin/env python

import os
import numpy as np
import cv2
import imutils

FNAME = os.path.expanduser('~/work/roborep/cv_test/video_shots/data/input/v.MOV')

cap = cv2.VideoCapture(FNAME)

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
					   qualityLevel = 0.3,
					   minDistance = 7,
					   blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
				  maxLevel = 2,
				  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0,255,(100,3))

i = 0
while True:
	ret,frame = cap.read()
	if not frame is None:
		frame = imutils.resize(frame, width=320)
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		if i == 0:
			# Create a mask image for drawing purposes
			mask = np.zeros_like(gray)
			p0 = cv2.goodFeaturesToTrack(gray, mask = None, **feature_params)

		else:


			# calculate optical flow
			p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray, p0, None, **lk_params)

			if not p1 is None:

				print 'Remaining:', len(p1)

				# Select good points
				good_new = p1[st==1]
				good_old = p0[st==1]

#				print good_new.shape, good_old.shape, good_new.dtype, good_old.dtype

				apply_warp = len(good_old) >= 4
				if apply_warp:
					H,matches=cv2.findHomography(good_old, good_new, cv2.RANSAC)
					M=H[0:2,:]
					frame = cv2.warpAffine(frame, M, (frame.shape[1], frame.shape[0]))
#				tr = cv2.estimateRigidTransform(good_old, good_new, True);

				# draw the tracks
				vecs = []
				for i,(new,old) in enumerate(zip(good_new,good_old)):
					a,b = new.ravel()
					c,d = old.ravel()
#					cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
					if apply_warp:
						cv2.circle(frame,(c,d),5,color[i].tolist(),-1)
					else:
						cv2.circle(frame,(a,b),5,color[i].tolist(),-1)

#				img = cv2.bitwise_and(frame, frame, mask=mask)
#				img = cv2.add(frame, mask)

				cv2.imshow('frame', frame)
				k = cv2.waitKey(30) & 0xff
				if k == 27:
					break

#				p0 = good_new.reshape(-1,1,2)

#				if len(p0) < 10:
#				if apply_warp:
#					gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				p0 = cv2.goodFeaturesToTrack(gray, mask = None, **feature_params)

			else:
				print 'No motion'
				if len(p0) == 0:
					p0 = cv2.goodFeaturesToTrack(gray, mask = None, **feature_params)
					mask = np.zeros_like(gray)
					print 'New vector:', len(p0)


		# Now update the previous frame and previous points
		old_gray = gray.copy()
		i += 1

cv2.destroyAllWindows()
cap.release()
