#!/usr/bin/env python3

import os
import numpy as np
import cv2
import glob
import time
import pickle

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

vs = cv2.VideoCapture('http://hubee.local:8090/?action=stream')
time.sleep(1.0)

if os.path.exists('imgpoints.json'):
	imgpoints = pickle.load(open('imgpoints.pkl'))

else:
	while len(imgpoints) < 50:
		(grabbed, img) = vs.read()
		if grabbed:

			gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

			# Find the chess board corners
			ret, corners = cv2.findChessboardCorners(gray, (11,6),None)
			print(ret, len(imgpoints))

			# If found, add object points, image points (after refining them)
			if ret == True:
				objpoints.append(objp)

				corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
				imgpoints.append(corners2)

				# Draw and display the corners
				img = cv2.drawChessboardCorners(img, (11, 6), corners2,ret)
				cv2.imshow('calibration',img)
			cv2.imshow('source',gray)
			cv2.waitKey(100)
		else:
			print('Not grabbed')

pickle.dump(imgpoints, open('imgpoints.pkl', 'w'))



ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

while True:
	(grabbed, img) = vs.read()
	if grabbed:
		h,  w = img.shape[:2]
		newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
		# undistort
		dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

		# crop the image
		x,y,w,h = roi
		dst = dst[y:y+h, x:x+w]
		cv2.imshow('cropped',dst)

		# undistort
		mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
		dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

		# crop the image
		x,y,w,h = roi
		dst = dst[y:y+h, x:x+w]
		cv2.imshow('undistorted',dst)


cv2.destroyAllWindows()

