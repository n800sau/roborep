import os, sys, imghdr
import cv2

SRC = 'images'
INPUT = 'output'
#OUTPUT = 'processed'
TAGGED = 'tagged'

src_files = {}
for root, dirs, files in os.walk(SRC, followlinks=True):
	for fname in files:
		src_files[os.path.basename(fname)] = os.path.join(root, fname)

for root, dirs, files in os.walk(INPUT, followlinks=True):
	for fname in files:
		fname = os.path.join(root, fname)
		print os.path.basename(fname), imghdr.what(fname)
		if imghdr.what(fname) in ('png', 'jpeg'):
			image = cv2.imread(fname)
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)[1]
			(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			# loop over the contours
			n_areas = 0
			for c in sorted(cnts, key=lambda v: cv2.contourArea(v), reverse=True)[:1]:
				area = cv2.contourArea(c)
				if area > 9000:
					n_areas += 1
					print 'area:', area
#					(x, y, w, h) = cv2.boundingRect(c)
#					cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
			bname = os.path.basename(fname)
#			cv2.imwrite(os.path.join(OUTPUT, bname), image)
			if n_areas == 1:
				dfname = os.path.join(TAGGED, 'car', bname)
			else:
				dfname = os.path.join(TAGGED, 'nocar', bname)
			if not os.path.exists(os.path.dirname(dfname)):
				os.makedirs(os.path.dirname(dfname))
			os.symlink(os.path.abspath(src_files[os.path.basename(fname)]), dfname)
