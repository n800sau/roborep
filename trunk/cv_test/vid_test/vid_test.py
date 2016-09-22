#!/usr/bin/env python

import sys, os
import cv2
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.cluster import KMeans
from resultsmontage import ResultsMontage


#for f in dir(cv2):
#<->print f
#sys.exit()
ifname = os.path.expanduser('~/work/roborep/cv_test/cat_tests/cat.avi')

EXTRACTOR = "ORB"
MODIFIED = False
OUTMODE = 'blank'
DETECTOR = 'Dense'
#DETECTOR = 'SURF'
DETECTOR_OPTS = {
	'Dense': {
	"initXyStep": 20,
	},
}

out = None
fps = 25

#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(ifname)

# Define the codec and create VideoWriter object
fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')


detector = cv2.FeatureDetector_create(DETECTOR)
for k,v in DETECTOR_OPTS.get(DETECTOR, {}).items():
	detector.setInt(k, v)

extractor = cv2.DescriptorExtractor_create(EXTRACTOR)
matcher = cv2.DescriptorMatcher_create('BruteForce-Hamming')

colors = ((255,0,0), (0,255,0), (0,0,255))

def drawMatches(image, kpsA, kpsB, matches):
	# loop over the matches
	for (trainIdx, queryIdx) in matches:
		# generate a random color and draw the match
		color = np.random.randint(0, high=255, size=(3,))
		ptA = (int(kpsA[queryIdx].pt[0]), int(kpsA[queryIdx].pt[1]))
		ptB = (int(kpsB[trainIdx].pt[0]), int(kpsB[trainIdx].pt[1]))
		cv2.line(image, ptA, ptB, color, 2)


n = 0
while cap.isOpened():
	ret, frame = cap.read()
	if ret==True:
		# same output video and frame size is important
		frame = cv2.resize(frame, (640, 360))

		if n == 0:
			sz = list(reversed(frame.shape[:2]))
#			out_gray = cv2.VideoWriter('output_%s_gray.avi' % EXTRACTOR, fourcc, fps, tuple(sz), 0)
			sz[0] *= 2
			out = cv2.VideoWriter('output_%s_%s_%s_matches_1.avi' % (DETECTOR, EXTRACTOR, OUTMODE), fourcc, fps, tuple(sz))

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		h,s,v = cv2.split(hsv)
		v = np.full_like(v, 128)
		frame1 = cv2.cvtColor(cv2.merge((h,s,v)), cv2.COLOR_HSV2BGR)
		gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)


#		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#		gray = cv2.GaussianBlur(gray, (61, 61), 0)

		kps = detector.detect(gray)
		(kps, descs) = extractor.compute(gray, kps)
		if MODIFIED:
			eps = 1e-7
			descs /= (descs.sum(axis=1, keepdims=True) + eps)
			descs = np.sqrt(descs)
		if n == 0:
			model = KNeighborsClassifier(n_neighbors=1, n_jobs=-1)
#			model = SVC(kernel="linear")
			clt = KMeans(n_clusters=len(colors), n_jobs=-1)
			clt.fit(descs)
			model.fit(descs, clt.labels_)
		else:
			rawMatches = matcher.knnMatch(last_descs, descs, 2)
			matches = []
			# loop over the raw matches
			for m in rawMatches:
				# ensure the distance passes David Lowe's ratio test
				if len(m) == 2 and m[0].distance < m[1].distance * 0.8:
					matches.append((m[0].trainIdx, m[0].queryIdx))

			predictions = model.predict(descs)
			groups = {}
			for i in range(len(predictions)):
				l = predictions[i]
				groups[l] = groups.get(l, {'kps': [], 'descs': []})
				groups[l]['kps'].append(kps[i])
				groups[l]['descs'].append(descs[i])

			if OUTMODE == 'blank':
				pimage = np.zeros(frame.shape, dtype='uint8')
				for l,m in groups.items():
					pimage = cv2.drawKeypoints(pimage, m['kps'], color=colors[l])
#				drawMatches(pimage, last_kps, kps, matches)
			else:
				pimage = cv2.drawKeypoints(frame, m['kps'], color=colors[l])
				drawMatches(pimage, last_kps, kps, matches)

			montage = ResultsMontage(frame.shape[:2], 2, 2)
			montage.addResult(frame)
			montage.addResult(pimage)
			out.write(montage.montage)


		last_kps = kps
		last_descs = descs

		n += 1
	else:
		print 'No more'
		break
else:
	print 'Can not open %s' % ifname

print 'processed %d frames' % (n-1)

# Release everything if job is finished
cap.release()
if not out is None:
	out.release()

