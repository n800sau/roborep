#!/usr/bin/env python3

import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.png')

for fname in images:
	img = cv2.imread(fname)
	(h, w) = img.shape[:2]
	nw = 320
	r = nw / float(w)
	img = cv2.resize(img, (nw, int(h * r)), cv2.INTER_AREA)
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

	features = cv2.goodFeaturesToTrack(gray, 25, 0.01, 10)
	features = np.int0(features)
	print('Features count:', features.size)


	if features.size > 0:
		for i in features:
			x,y = i.ravel()
			cv2.circle(img,(x,y),3,(0,0,255),-1)

		print('Write features')
		cv2.imwrite('images/features.jpg',img)

	# Find the chess board corners
	ret, corners = cv2.findChessboardCorners(gray, (9,6), flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
	print('Corners count:', corners.size)

	# If found, add object points, image points (after refining them)
	if ret == True:
		objpoints.append(objp)

		corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
		imgpoints.append(corners2)

		# Draw and display the corners
		img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
		print('Write corners')
		cv2.imwrite('images/corners.jpg',img)
