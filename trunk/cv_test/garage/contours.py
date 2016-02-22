#!/usr/bin/env python

import matplotlib
matplotlib.use('Agg')
import os, sys, glob, json, time, random
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
from skimage import feature
from skimage import exposure
from sklearn.cluster import KMeans
import numpy as np
from matplotlib import pyplot as plt
import mahotas
import pylab

time_mark = int(time.time())

img_path = os.path.join(os.path.expanduser('~/sshfs/asus/root/rus_hard/garage/2016-02-15'), '*.jpg')
out_path = os.path.join(os.path.dirname(__file__), 'output')

random.seed()

COLOR_COUNT = 20
colors = [(random.choice(range(0, 255, 30)), 250, 250) for i in range(COLOR_COUNT)]

def sort_contours(cnts, method="left-to-right"):
	# initialize the reverse flag and sort index
	reverse = False
	i = 0

	# handle if we need to sort in reverse
	if method == "right-to-left" or method == "bottom-to-top":
		reverse = True

	# handle if we are sorting against the y-coordinate rather than
	# the x-coordinate of the bounding box
	if method == "top-to-bottom" or method == "bottom-to-top":
		i = 1

	# construct the list of bounding boxes and sort them from top to
	# bottom
	boundingBoxes = [cv2.boundingRect(c) for c in cnts]
	(cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes),
		key=lambda b:b[1][i], reverse=reverse))

	# return the list of sorted contours and bounding boxes
	return (cnts, boundingBoxes)

def draw_contour(image, c, i, color, orient, put_test=True):
	# compute the center of the contour area and draw a circle
	# representing the center
	M = cv2.moments(c)
	if M['m00'] != 0:
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])

		cv2.drawContours(image, [c], -1, color, 2)
		if put_test:
			# draw the countour number on the image
			cv2.putText(image, "#{}{}".format(i, orient if orient else 'unknown'), (cX - 20, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

	# return the image with the contour number drawn on it
	return image

def write_image(image, fname, fnames):

	fn = os.path.join('images', fname)
	fnames.append(fn)
	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(image, width=80))

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

# right angle location:
# rt - right top
# rb - right bottom
# lt - left top
# lb - left bottom
def triangle_orientation(points, ttype = 'rt'):
	rs = ''
	if len(points) == 3:
		# triangle
		p = points // 60
		print p
		xx = p[:, 1]
		yy = p[:, 0]
		xmin = sum(xx == xx.min())
		xmax = sum(xx == xx.max())
		ymin = sum(yy == yy.min())
		ymax = sum(yy == yy.max())
		print xmin, xmax, ymin, ymax
		hor = 'r' if xmax == 2 and xmin == 1 else ''
		hor += 'l' if xmax == 1 and xmin == 2 else ''
		ver = 'b' if ymax == 2 and ymin == 1 else ''
		ver += 't' if ymax == 1 and ymin == 2 else ''
		if hor and ver:
			rs = hor + ver
	return rs

def car_crop(image):
	return image[90:, 70:270]

def process_image(fname, mask=None, save=True):
	bname = os.path.basename(fname)
	image = cv2.imread(fname)
	image = car_crop(image)
#	write_image(image, bname + '_image.png', fnames)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#	(H, hogImage) = feature.hog(gray, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), normalise=True, visualise=True)
	H = feature.hog(gray, orientations=9, pixels_per_cell=(8, 8), cells_per_block=(2, 2), normalise=True, visualise=False)
#	hogImage = exposure.rescale_intensity(hogImage, out_range=(0, 255))
#	hogImage = hogImage.astype("uint8")
#	write_image(hogImage, bname + '_hog.png', fnames)
	if save:
		json.dump(list(H), file(os.path.join('hog', bname + '.hog'), 'w'))
	return H

def process_image2(fname, mask=None):

	bname = os.path.basename(fname)
	image = cv2.imread(fname)

	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


	gray = cv2.medianBlur(gray, 3)

	v = np.median(gray)
	sigma = 0.33
	slower = int(max(0, (1.0 - sigma) * v))
	supper = int(min(255, (1.0 + sigma) * v))

	# 60 - min distance between centres
	circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1, 30, param1=supper, param2=30, minRadius=7, maxRadius=25)

	if not circles is None:

		circles = np.uint16(np.around(circles))
		print 'Found ', circles
		for i in circles[0,:]:
			# draw the outer circle
			cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
			# draw the center of the circle
			cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)

		write_image(image, bname + '_circles.png', fnames)

#		if not mask is None:
#			masked_gray = cv2.bitwise_and(gray, gray, mask = mask)

#		write_image(masked_gray, bname + '_masked.png', fnames)

		gray = cv2.Canny(gray, slower, supper)
		write_image(gray, bname + '_canny.png', fnames)


def process_image1(fname, mask=None):

	bname = os.path.basename(fname)

#	image = cv2.imread(fname)
#	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# load the image and initialize the accumulated edge image
	image = cv2.imread(fname)
##	accumEdged = np.zeros(image.shape[:2], dtype="uint8")

	# loop over the blue, green, and red channels, respectively
##	for chan in cv2.split(image):
		# blur the channel, extract edges from it, and accumulate the set
		# of edges for the image
##		chan = cv2.medianBlur(chan, 7)
#		chan = cv2.medianBlur(chan, 11)
##		edged = cv2.Canny(chan, 50, 150)
##		accumEdged = cv2.bitwise_or(accumEdged, edged)

	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	if not mask is None:
		gray = cv2.bitwise_and(gray, gray, mask = mask)

#	gray = cv2.medianBlur(gray, 3)

# for big triangles
#	gray = cv2.medianBlur(gray, 9)

#	gray = cv2.GaussianBlur(gray, (11, 11), 0)
#	squareKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
#	gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, squareKernel)
#	gray = cv2.erode(gray.copy(), None, iterations=3)


	gray = auto_canny(gray)

#	for triangles
#	(T, gray) = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

#	print 'T=', T

#	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
#	gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
#	gray = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel)


#	for triangles
#	gray = cv2.erode(gray, None, iterations=6)
#	for triangles
#	gray = cv2.dilate(gray, None, iterations=5)

#	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
#	gray = cv2.morphologyEx(gray, cv2.MORPH_GRADIENT, kernel)

#	for triangles
#	gray = auto_canny(gray)


#	accumEdged = cv2.Canny(gray, 50, 150)

#	write_image(accumEdged, bname + '_edged.png', fnames)

	accumEdged = gray

	# find contours in the accumulated image, keeping only the largest ones
	(cnts, _) = cv2.findContours(accumEdged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#	cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]

	orig = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#	orig = image.copy()

	found = False

	# loop over the (unsorted) contours and draw them
	for (i, c) in enumerate(cnts):
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.06 * peri, True)
		if len(approx) == 3 and False:
			(x, y, w, h) = cv2.boundingRect(approx)
			if w < h:
				area = cv2.contourArea(approx)
				if area > 10000 and area < 20000:
					orient = triangle_orientation(approx[:,0])
					if orient != 'rb':
						print '%d: n points: %d, area: %.2f, orient: %s, color: %s, %dx%d' % (i, len(approx), area, orient, colors[i%COLOR_COUNT], w, h)
						orig = draw_contour(orig, approx, i, colors[i%COLOR_COUNT], orient)
						found = True
		elif len(c) > 30:
			area = cv2.contourArea(approx)
			if area > 100 and area < 1000:
				print '%d: n points: %d, area: %.2f' % (i, len(c), area)
#				hull = cv2.convexHull(c)
#				hullArea = cv2.contourArea(hull)
#				solidity = area / float(hullArea)
#				circle = cv2.minEnclosingCircle(c)

				ellipse = cv2.fitEllipse(c)
				mask = np.zeros(image.shape[:2], dtype="uint8")
				cv2.ellipse(mask, ellipse, 255, -1)
				solidity = (area / np.sum(mask > 0)) if np.sum(mask > 0) else 0

				print 'solidity:', solidity, 'mask sum:', np.sum(mask > 0), 'area=', area
		# mask
#				print 'solidity:', solidity, 'circle:', circle, 'ellipse:', ellipse
#				print 'circle area:', circle[1] * 2, 'area:', area, 'solidity:', area/float(circle[1] * 2)
				if solidity > 0.7:
					orig = draw_contour(orig, c, i, colors[i%COLOR_COUNT], '', put_test=False)
					cv2.ellipse(orig, ellipse, (0, 255, 0), 2)
#					cv2.circle(orig, (int(circle[0][0]),int(circle[0][1])) , int(circle[1]), (0, 255, 0), 2)
					found = True

	if found:
		orig = cv2.cvtColor(orig, cv2.COLOR_HSV2BGR)
		write_image(gray, bname + '_gray.png', fnames)
		write_image(orig, bname + '_unsorted.png', fnames)

	# sort the contours according to the provided method
##	(cnts, boundingBoxes) = sort_contours(cnts)

	# loop over the (now sorted) contours and draw them
##	for (i, c) in enumerate(cnts):
##		peri = cv2.arcLength(c, True)
##		approx = cv2.approxPolyDP(c, 0.01 * peri, True)
##		print i, ':sorted n points:', len(approx)
#		if len(approx) == 3:
##		draw_contour(image, approx, i)

##	write_image(image, bname + '_sorted.png', fnames)

	print '-' * 5, bname, '\n\n\n'


SAMPLE_FNAME = 'sample.json'

if os.path.exists(SAMPLE_FNAME):
	flist = json.load(file(SAMPLE_FNAME))
else:
	flist = glob.glob(img_path)
#	flist = random.sample(flist, min(len(flist), 10))
	random.shuffle(flist)
	json.dump(flist, file(SAMPLE_FNAME, 'w'), indent=2)

#fgbg = cv2.BackgroundSubtractorMOG()

fnames = []

featurelist = []

#i = 0
#for fn in flist:
#	bname = os.path.basename(fn)
#	frame = cv2.imread(fn, 0)
#	pylab.gray()
#	pylab.subplot(131)
#	pylab.imshow(frame)
#	edge_sobel = mahotas.sobel(frame)
#	pylab.subplot(132)
#	pylab.imshow(edge_sobel)
#	pylab.savefig('/tmp/fig.png', bbox_inches='tight', format='png')
#	write_image(cv2.imread('/tmp/fig.png'), bname + '_sabel.png', fnames)
#	
#	i += 1
#	if i > 10:
#		break

# make background
#for fn in flist:
#	frame = cv2.imread(fn)
#	fgmask = fgbg.apply(frame)
#	bname = os.path.basename(fn)
#write_image(fgmask, bname + '_bgmask.png', fnames)


processed = []
i = 0
for fn in flist:
	bname = os.path.basename(fn)
	hogname = os.path.join('hog', bname + '.hog')
	i += 1
	if os.path.exists(hogname):
		f = json.load(file(hogname))
	else:
		print 'Not Found !!!!!!!!!!!!!!!!!!!!!!!'
		f = process_image(fn)
	featurelist.append(f)
	processed.append(fn)
#	process_image(fn, mask=fgmask)
#	if len(fnames) >= 20:
#		break
#	if i >= 40:
#		break

clt = KMeans(n_clusters=3)
clt.fit(featurelist)

for fn in glob.glob('car/*.jpg'):
	bname = os.path.basename(fn)
	image = cv2.imread(fn)
	hog = process_image(fn, save=False)
	print bname, clt.predict([hog])


#fnames = dict([(l,[]) for l in clt.labels_])

files = {}
for fn, label in zip(processed, clt.labels_):
	files[label] = files.get(label, [])
	if len(files[label]) < 5:
		files[label].append(fn)

for label,lfn in files.items():
	for fn in lfn:
		bname = os.path.basename(fn)
		image = cv2.imread(fn)
		cv2.putText(image, "cl:{}".format(label), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
		write_image(image, bname + '_image.png', fnames)

#for fn in flist:

#	process_image(fn)
#	if len(fnames) >= 10:
#		break

print 'Found %d files in (%d) (stopped at %d)' % (len(fnames), len(flist), i)


json.dump({
	'files': random.sample(fnames, min(20, len(fnames))),
	'title': os.path.splitext(os.path.basename(__file__))[0],
	'time_mark': time_mark,
}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)
