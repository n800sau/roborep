#!/usr/bin/env python

import sys, os, glob
import cv2, imutils
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.cluster import KMeans
from resultsmontage import ResultsMontage

SRCDIR = os.path.expanduser('~/sshfs/asus/root/rus_hard/scaremaster/2016-04-20')

OUT_WIDTH = 640

EXTRACTOR = 'ORB'
MODIFIED = False
# gray, hsv
MODE = 'hsv'

# Define the codec and create VideoWriter object
fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')
fps = 5


detector = cv2.FeatureDetector_create("Dense")
detector.setInt("initXyStep", 10)
extractor = cv2.DescriptorExtractor_create(EXTRACTOR)

colors = ((255,0,0), (0,255,0), (0,0,255))

flist = glob.glob(os.path.join(SRCDIR, '*.jpg'))
fnum = len(flist)
print 'Count: %d' % fnum

i = 0
for fname in flist:
	frame = cv2.imread(fname)
	if frame is None:
		print >>sys.stderr, 'Bad frame'
	else:
		frame = imutils.resize(frame, width=OUT_WIDTH)

		if MODE == 'gray':
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		elif MODE == 'hsv':
			hsvimage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			gray,s,v = cv2.split(hsvimage)
		else:
			gray = frame.copy()

		gray = cv2.GaussianBlur(gray, (51, 51), 0)
#		gray = cv2.blur(gray, (11, 11))

		kps = detector.detect(gray)
		(kps, descs) = extractor.compute(gray, kps)

		if MODIFIED:
			eps = 1e-7
			descs /= (descs.sum(axis=1, keepdims=True) + eps)
			descs = np.sqrt(descs)

		if i == 0:
			sz = list(reversed(frame.shape[:2]))
#			out_gray = cv2.VideoWriter('output_%s_gray.avi' % EXTRACTOR, fourcc, fps, tuple(sz), 0)
			sz[0] *= 2
			out = cv2.VideoWriter('output_%s_%s.avi' % (EXTRACTOR, MODE), fourcc, fps, tuple(sz))

			model = SVC(kernel="linear")
#			model = KNeighborsClassifier(n_neighbors=3, n_jobs=-1)
			clt = KMeans(n_clusters=len(colors), n_jobs=-1)
			clt.fit(descs)
			model.fit(descs, clt.labels_)
		else:
			predictions = model.predict(descs)
			groups = {}
			for j in range(len(predictions)):
				l = predictions[j]
				groups[l] = groups.get(l, {'kps': [], 'descs': []})
				groups[l]['kps'].append(kps[j])
				groups[l]['descs'].append(descs[j])

			pimage = np.zeros(frame.shape, dtype='uint8')
			for l,m in groups.items():
				pimage = cv2.drawKeypoints(pimage, m['kps'], color=colors[l])

#			out_gray.write(gray)

			montage = ResultsMontage(frame.shape[:2], 2, 2)
#			montage.addResult(gray)
			montage.addResult(frame)
			montage.addResult(pimage)
			out.write(montage.montage)
		i += 1
		print '%2.2d%%           \r' % int(i/float(fnum) * 100),
		sys.stdout.flush()

print 'processed %d frames' % (i-1)

out.release()
#out_gray.release()
