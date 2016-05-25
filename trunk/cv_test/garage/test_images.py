#!/usr/bin/env python

import os, glob, json, random, shutil
import cv2
import imutils
from skimage.filters import threshold_adaptive
from skimage import measure
import numpy as np
from matplotlib import pyplot as plt
from make_hogs import find_orig_file
from sklearn.cluster import KMeans
from skimage import feature

img_paths = [
	os.path.join(os.path.expanduser('traindata/car_inside'), '*.jpg'),
	os.path.join(os.path.expanduser('traindata/nocar'), '*.jpg'),
]
out_path = os.path.join(os.path.dirname(__file__), 'output')

fnames = []


def write_image(image, fname, fnames):

	fn = os.path.join('images', fname)
	fnames.append(fn)
	cv2.imwrite(os.path.join(out_path, fn), imutils.resize(image, width=160))

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

def process_image(fname):

	pfx = os.path.splitext(os.path.basename(fname))[0]

	plt.ioff()

	detector = cv2.FeatureDetector_create("Dense")
	detector.setInt("initXyStep", 6)
	extractor = cv2.DescriptorExtractor_create("SURF")

	image = cv2.imread(fname)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#	blobs = feature.blob_dog(gray, threshold=.5, max_sigma=40)
#	bimage = image.copy()
#	for i in range(blobs.shape[0]):
#		blob = blobs[i]
#		print blob
#		cv2.circle(bimage, (int(blob[0]), int(blob[1])), int(blob[2]), (128, 255, 0), -1)
#	write_image(bimage, pfx + '_blobs.jpg', fnames)

#	labimage = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
#	l,a,b = cv2.split(labimage)
#	cl = []
#	for t in (l, a, b):
#		t = cv2.GaussianBlur(t, (11, 11), 0)
#		cl.append(((t - t.min()) * 256. / (t.max() - t.min())).astype('uint8'))
#	labimage = cv2.merge(cl)
#	nimage = cv2.cvtColor(labimage, cv2.COLOR_LAB2BGR)
#	write_image(nimage, pfx + '_labimage.jpg', fnames)

#	hsvimage = cv2.cvtColor(nimage, cv2.COLOR_BGR2LAB)
#	h,s,v = cv2.split(hsvimage)
#	write_image(h, pfx + '_hueimage.jpg', fnames)

#	hist = cv2.calcHist([h], [0], None, [256], [0, 256])
	# plot the histogram
#	plt.figure(figsize=(2,2))
#	plt.title("Hue Histogram")
#	plt.xlabel("Bins")
#	plt.ylabel("# of Pixels")
#	plt.plot(hist)
#	plt.xlim([0, 256])
#	fn = os.path.join('images', pfx + '_hue_histogram.png')
#	fnames.append(fn)
#	plt.savefig(os.path.join(out_path, fn), bbox_inches='tight', format='png')


#	gray = v
#	for i in range(1):
#		gray = cv2.medianBlur(gray, 3)
#	for i in range(1):
#		gray = cv2.GaussianBlur(gray, (11, 11), 0)

#	write_image(gray, pfx + '_blurred.jpg', fnames)

#	mask = (gray > 170).astype("uint8")
#	cv2.bitwise_and(gray, gray, mask = mask)

	# construct a grayscale histogram
#	hist = cv2.calcHist([gray], [0], None, [16], [100, 256])


	# plot the histogram
#	plt.figure(figsize=(2,2))
#	plt.title("Grayscale Histogram")
#	plt.xlabel("Bins")
#	plt.ylabel("# of Pixels")
#	plt.plot(hist)
#	plt.xlim([0, 16])

#	fn = os.path.join('images', pfx + '_histogram.png')
#	fnames.append(fn)
#	plt.savefig(os.path.join(out_path, fn), bbox_inches='tight', format='png')

	# normalize the histogram
#	hist /= hist.sum()

	# plot the normalized histogram
#	plt.figure(figsize=(2,2))
#	plt.title("Grayscale Histogram (Normalized)")
#	plt.xlabel("Bins")
#	plt.ylabel("% of Pixels")
#	plt.plot(hist)
#	plt.xlim([0, 256])


#	fn = os.path.join('images', pfx + '_histogram_norm.png')
#	fnames.append(fn)
#	plt.savefig(os.path.join(out_path, fn), bbox_inches='tight', format='png')

#	write_image(v, pfx + '_value.jpg', fnames)

#	gX = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
#	gY = cv2.Sobel(gray, cv2.CV_64F, 0, 1)

#	gXmin = gX.min()
#	gXmax = gX.max()
#	gXimage = ((gX - gXmin) / ((gXmax - gXmin) / 256)).astype('uint8')
#	write_image(gXimage, pfx + '_gX.jpg', fnames)

#	gYmin = gY.min()
#	gYmax = gY.max()
#	gYimage = ((gY - gYmin) / ((gYmax - gYmin) / 256)).astype('uint8')
#	write_image(gYimage, pfx + '_gY.jpg', fnames)


#	mag = np.sqrt((gX ** 2) + (gY ** 2))
#	orientation = np.arctan2(gY, gX) * (180 / np.pi) % 180

#	omin = orientation.min()
#	omax = orientation.max()
#	orimage = ((orientation - omin) / ((omax - omin) / 256)).astype('uint8')

#	write_image(orimage, pfx + '_orientation.jpg', fnames)

	kps = detector.detect(gray)
	(kps, descs) = extractor.compute(gray, kps)
	print("[INFO] # of keypoints detected: {}".format(len(kps)))
	print("[INFO] feature vector shape: {}".format(descs.shape))

	clt = KMeans(n_clusters=3)
	clt.fit(descs)
	groups = {}
	for i in range(len(clt.labels_)):
		l = clt.labels_[i]
		groups[l] = groups.get(l, {'kps': [], 'descs': []})
		groups[l]['kps'].append(kps[i])
		groups[l]['descs'].append(descs[i])

	kimage = image.copy()

	colors = ((255,0,0), (0,255,0), (0,0,255))
	for l,m in groups.items():
		kimage = cv2.drawKeypoints(kimage, m['kps'], color=colors[l])

	write_image(kimage, pfx + '_keypoints.jpg', fnames)

#	for si in range(1, 10):
	si = 3.3
	(cnts, _) = cv2.findContours(auto_canny(gray, sigma=si/10.), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	cntlist = []
	boxlist = []
	for c in cnts:
		(x, y, w, h) = cv2.boundingRect(c)
		if w > 30 or h > 30:

			area = cv2.contourArea(c)
			if area > 900:
				cntlist.append(c)
				boxlist.append((x, y, w, h))

	cv2.drawContours(image, cntlist, -1, (0, 255, 0), 2)
	print "Found {} contours".format(len(cntlist))
	for b in boxlist:
		cv2.rectangle(image, (b[0], b[1]), (b[0]+b[2], b[1]+b[3]), (0, 0, 255), 1)
	write_image(image, pfx + '_%d_contours.jpg' % si, fnames)

	print '%d files' % len(fnames)

	json.dump({
		'files': fnames,
		'title': os.path.splitext(os.path.basename(__file__))[0],
	}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)

for fi in range(len(img_paths)):

	fn = os.path.join('cache', 'img%d.jpg' % fi)

	if not os.path.exists(fn):

		sfn = random.sample(glob.glob(img_paths[fi]), 1)[0]
		sfn = find_orig_file(os.path.basename(sfn))
		shutil.copyfile(sfn, fn)

	process_image(fn)
