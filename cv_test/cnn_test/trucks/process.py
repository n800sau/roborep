#!/usr/bin/env python

import os, sys
import cv2
import numpy as np
from imutils import paths, resize
from sklearn.neighbors import KNeighborsClassifier
from sklearn.cluster import KMeans
from skimage import measure

INPUT = 'input'
OUTPUT = 'output'

#detector = cv2.FeatureDetector_create("Dense")
#detector.setInt("initXyStep", 6)
#DETECTOR = "SIFT"
#EXTRACTOR = "SIFT"

DETECTOR = "ORB"
EXTRACTOR = "ORB"

detector = cv2.FeatureDetector_create(DETECTOR)
extractor = cv2.DescriptorExtractor_create(EXTRACTOR)

colors = ((0,0,255), (0,255,0), (255,0,0))

AVGFNAME = 'avg.npy'
avg = None
if os.path.exists(AVGFNAME):
	avg = np.load(AVGFNAME)

def auto_canny(image, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(image)

	# apply automatic Canny edge detection using the computed median
	slower = int(max(0, (1.0 - sigma) * v))
	supper = int(min(255, (1.0 + sigma) * v))
#	print('lower=%s, upper=%s' % (slower, supper))
	edged = cv2.Canny(image, slower, supper)

	# return the edged image
	return edged

for ifname in paths.list_images(INPUT):
	print ifname
	image = cv2.imread(ifname)
#	resized = resize(image, width=360)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	if not avg is None:
#		gray = cv2.absdiff(gray, avg)
		gray = cv2.absdiff(gray, cv2.convertScaleAbs(avg))
		(T, threshInv) = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
		cv2.imwrite(os.path.join(OUTPUT, 'avg_' + os.path.basename(ifname)), resize(threshInv, width=320))
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (90, 90))
		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_CLOSE, kernel)
		cv2.imwrite(os.path.join(OUTPUT, 'close_' + os.path.basename(ifname)), resize(oimage, width=320))
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (90, 90))
		oimage = cv2.morphologyEx(oimage, cv2.MORPH_OPEN, kernel)
		cv2.imwrite(os.path.join(OUTPUT, 'open_' + os.path.basename(ifname)), resize(oimage, width=320))
		(cnts, _) = cv2.findContours(oimage.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		gray = cv2.bitwise_and(gray, gray, mask=oimage)
		blurred = cv2.GaussianBlur(gray, (17, 17), 0)
		kps = detector.detect(blurred)
		(kps, descs) = extractor.compute(blurred, kps)
#		clt = KMeans(n_clusters=3)
#		clt.fit(descs)
#		for i in np.unique(clt.labels_):
#			print i, ':', len(np.where(clt.labels_ == i)[0])
#			image = cv2.drawKeypoints(image, np.take(kps, np.where(clt.labels_ == i)[0]), color=colors[i])
		image = cv2.drawKeypoints(image, kps, color=colors[0])
		cv2.imwrite(os.path.join(OUTPUT, os.path.basename(ifname)), resize(image, width=320))
		kmask = np.zeros(image.shape[:2], dtype=np.uint8)
#		print '1 kmask', kmask.shape
		kmask = cv2.drawKeypoints(kmask, kps, color=(255,))
		kmask = cv2.cvtColor(kmask, cv2.COLOR_BGR2GRAY)
		kmask = cv2.GaussianBlur(kmask, (7, 7), 0)
		(T, kmask) = cv2.threshold(kmask, 3, 255, cv2.THRESH_BINARY)
		tot = kmask.sum()
		print 'tot=', tot
#		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 17))
#		for i in range(30):
#			kmask = cv2.morphologyEx(kmask, cv2.MORPH_CLOSE, kernel)
#		for i in range(10):
#			kmask = cv2.morphologyEx(kmask, cv2.MORPH_OPEN, kernel)
#		print '2 kmask', kmask.shape
		cv2.imwrite(os.path.join(OUTPUT, 'kmask_' + os.path.basename(ifname)), resize(kmask, width=320))

#		blurred = cv2.GaussianBlur(gray, (17, 17), 0)
#		(T, threshInv) = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
#		for i in range(10, 100, 10):
#		kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (i, i))
#			kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (i, i))

#		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_OPEN, kernel) # 1
#		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_CLOSE, kernel) # 2
#			oimage = cv2.morphologyEx(threshInv, cv2.MORPH_GRADIENT, kernel)
#		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_TOPHAT, kernel)
#		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_BLACKHAT, kernel)
#		oimage = cv2.erode(threshInv.copy(), None, iterations=i + 1)

#			cv2.imwrite(os.path.join(OUTPUT, str(i) + '_' + os.path.basename(ifname)), resize(oimage, width=320))
#		for c in cnts:
#			M = cv2.moments(c)
#			cX = int(M["m10"] / M["m00"])
#			cY = int(M["m01"] / M["m00"])
#			cv2.circle(oimage, (cX, cY), 10, (0, 255, 0), -1)
#		cv2.imwrite(os.path.join(OUTPUT, 'result_' + os.path.basename(ifname)), resize(oimage, width=320))
	else:

		print 'No avg found'
#	gray = cv2.blur(gray, (5, 5))
		blurred = cv2.GaussianBlur(gray, (17, 17), 0)
	# remove shadow
		(T, threshInv) = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
#	cv2.imwrite(os.path.join(OUTPUT, os.path.basename(ifname)), resize(threshInv, width=320))
		for i in range(10, 100, 10):
#		kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (i, i))
			kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (i, i))

#		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_OPEN, kernel) # 1
#		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_CLOSE, kernel) # 2
			oimage = cv2.morphologyEx(threshInv, cv2.MORPH_GRADIENT, kernel)
#		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_TOPHAT, kernel)
#		oimage = cv2.morphologyEx(threshInv, cv2.MORPH_BLACKHAT, kernel)
#		oimage = cv2.erode(threshInv.copy(), None, iterations=i + 1)

			cv2.imwrite(os.path.join(OUTPUT, str(i) + '_' + os.path.basename(ifname)), resize(oimage, width=320))

#	kps = detector.detect(gray)
#	(kps, descs) = extractor.compute(gray, kps)
#	clt = KMeans(n_clusters=3)
#	clt.fit(descs)
#	for i in np.unique(clt.labels_):
#		print i, ':', len(np.where(clt.labels_ == i)[0])
#		image = cv2.drawKeypoints(image, np.take(kps, np.where(clt.labels_ == i)[0]), color=colors[i])
#	image = cv2.drawKeypoints(image, kps, color=colors[0])

#	cv2.imwrite(os.path.join(OUTPUT, os.path.basename(ifname)), resize(image, width=320))

#	for i in range(10, 200, 20):
#		(T, threshInv) = cv2.threshold(blurred, i, 255, cv2.THRESH_BINARY)
#		oimage = threshInv

#	blurred = cv2.bitwise_and(blurred, blurred, mask=threshInv)
#	gX = cv2.Sobel(gray, ddepth=cv2.CV_64F, dx=1, dy=0)
#	gY = cv2.Sobel(gray, ddepth=cv2.CV_64F, dx=0, dy=1)
#	gX = cv2.convertScaleAbs(gX)
#	gY = cv2.convertScaleAbs(gY)
#	oimage = cv2.addWeighted(gX, 0.5, gY, 0.5, 0)
#	oimage = auto_canny(oimage)

#		(cnts, _) = cv2.findContours(oimage.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#		cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
#		for c in cnts:
#			peri = cv2.arcLength(c, True)
#			approx = cv2.approxPolyDP(c, 0.01 * peri, True)
#			area = cv2.contourArea(c)
#			if len(approx) == 4 and area > 6:
#				print 'approx size=', len(approx), ",", area
#				cv2.drawContours(image, [c], -1, (0, 255, 255), 5)
#				(x, y, w, h) = cv2.boundingRect(approx)
#				print 'Found', x, y, w, h
#		cv2.putText(resized, "Rectangle", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
#				cv2.imwrite(os.path.join(OUTPUT, str(i) + '_' + os.path.basename(ifname)), resize(image, width=320))
