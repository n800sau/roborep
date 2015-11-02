import os
from picamera.array import PiRGBArray
import cv2
import imutils
from utils import dbprint
import numpy as np
from matplotlib import pyplot as plt

# "FAST"
# "STAR"
# "SIFT"
# "SURF"
# "ORB"
# "MSER"
# "GFTT"
# "HARRIS"
# "Dense"
# "SimpleBlob"
# "GridFAST"
# "GridSTAR"
# "GridSIFT"
# "GridSURF"
# "GridORB"
# "GridMSER"
# "GridGFTT"
# "GridHARRIS"
# "GridDense"
# "GridSimpleBlob"
# "PyramidFAST"
# "PyramidSTAR"
# "PyramidSIFT"
# "PyramidSURF"
# "PyramidORB"
# "PyramidMSER"
# "PyramidGFTT"
# "PyramidHARRIS"
# "PyramidDense"
# "PyramidSimpleBlob"

# "FAST/0" - FastFeatureDetector
# "STAR" - StarFeatureDetector
# "SIFT"* - SIFT (nonfree module)
# "SURF" - SURF (nonfree module)
# "ORB"+ - ORB
# "BRISK" - BRISK
# "MSER" - MSER
# "GFTT"+ - GoodFeaturesToTrackDetector
# "HARRIS" - GoodFeaturesToTrackDetector with Harris detector enabled
# "Dense" - DenseFeatureDetector
# "SimpleBlob" - SimpleBlobDetector
DETECTOR = 'ORB'
#DETECTOR = 'GFTT'
NORM = cv2.NORM_HAMMING

#for sift, surf
#NORM = cv2.NORM_L2



# SIFT/0, SURF*, BRIEF+, BRISK*, ORB+, FREAK*
EXTRACTOR = 'ORB'
#EXTRACTOR = 'BRIEF'


FLANN_INDEX_KDTREE = 1  # bug: flann enums are missing
FLANN_INDEX_LSH    = 6


MATCHER = 'BruteForce-Hamming'
#MATCHER = 'Flann'

def set_params(camera, **params):
	camera.resolution = params.get('resolution', (320, 240))
	camera.exposure_mode = 'auto'
	camera.brightness = 70
	camera.contrast = 70

def update_img(camera, fname=None, **params):
	fname = os.path.join(os.path.expanduser('~/public_html'), fname or 'picam_0.jpg')
	set_params(camera, **params)
	camera.capture(fname, use_video_port=False)

def capture_cvimage(camera, **params):
	# initialize the camera and grab a reference to the raw camera capture
	stream = PiRGBArray(camera)
	set_params(camera, **params)
	# grab an image from the camera
	camera.capture(stream, format="bgr", use_video_port=True)
	return stream.array

class FeatureProcess(object):

	def __init__(self, camera=None):
		self.camera = camera
		self.img = None
		self.cv_det = cv2.FeatureDetector_create(DETECTOR)
		self.cv_desc = cv2.DescriptorExtractor_create(EXTRACTOR)
		self.matcher = cv2.DescriptorMatcher_create(MATCHER)
#		flann_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
#		flann_params= dict(algorithm = FLANN_INDEX_LSH,
#			table_number = 6, # 12
#			key_size = 12,     # 20
#			multi_probe_level = 1) #2
#		self.matcher = cv2.FlannBasedMatcher(flann_params, {})  # bug : need to pass empty dict (#1329)


	def filterMatches(self, kp, matches, ratio = 0.75):
		mkp1, mkp2 = [], []
		for m in matches:
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				m = m[0]
				mkp1.append(self.kp[m.queryIdx])
				mkp2.append(kp[m.trainIdx])
		p1 = np.float32([kp.pt for kp in mkp1])
		p2 = np.float32([kp.pt for kp in mkp2])
		return p1, p2, zip(mkp1, mkp2)

	def percent(self, frame=None):
		rs = None
		try:
			if frame is None and self.camera:
				frame = capture_cvimage(self.camera)
			if not frame is None:
				frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
				if self.img is None:
					self.img = frame
					self.kp = self.cv_det.detect(self.img)
					self.kp, self.desc = self.cv_desc.compute(self.img, self.kp)
					self.kpl = len(self.kp)
					dbprint('kp type:%s, desc type=%s' % (type(self.kp), type(self.desc)))
				else:
					kp = self.cv_det.detect(frame)
					kp, desc = self.cv_desc.compute(frame, kp)
					matches = self.matcher.knnMatch(self.desc, trainDescriptors=desc, k=2)
					p1,p2,pairs = self.filterMatches(kp, matches)
					lp = len(pairs)
					percent = ((lp * 100) / self.kpl) if self.kpl else 0
					rs = {
						'frame': frame,
						'percent': percent
					}
					if len(p1) >= 4:
						H, status = cv2.findHomography(p1, p2, cv2.RANSAC, 2.0)
						dbprint('%d / %d  inliers/matched' % (np.sum(status), len(status)))
						pg = [p[0] for p in zip(p2, status) if p[1][0]]
						pnt = np.average(pg, axis=0)
						rs['point'] = pnt
						if not pnt is None:
							cv2.circle(frame, (pnt[0], pnt[1]), max(20, frame.shape[0] / 20), (0, 0, 255), -1)
		except cv2.error, e:
			dbprint('cv2 error: %s' % e)
		return rs

class ImageSearch(object):

	def __init__(self, camera=None):
		self.camera = camera
		self.idata = {}
		self.cv_det = cv2.FeatureDetector_create(DETECTOR)
		self.cv_desc = cv2.DescriptorExtractor_create(EXTRACTOR)
		self.matcher = cv2.DescriptorMatcher_create(MATCHER)

	def add_image(self, name, fname):
		img = cv2.imread(fname, 0)
		kp = self.cv_det.detect(img)
		kp, desc = self.cv_desc.compute(img, kp)
		self.idata[name] = {
			'kp': kp,
			'desc': desc,
			'kpl': len(kp),
		}

	def image_names(self):
		return self.idata.keys()

	def filter_matches(self, name, kp, matches, ratio = 0.75):
		ikp = self.idata[name]['kp']
		mkp1, mkp2 = [], []
		for m in matches:
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				m = m[0]
				mkp1.append(ikp[m.queryIdx])
				mkp2.append(kp[m.trainIdx])
		p1 = np.float32([kp.pt for kp in mkp1])
		p2 = np.float32([kp.pt for kp in mkp2])
		return p1, p2, zip(mkp1, mkp2)

	def _find_image(self, name, gray):
		rs = None
		idesc = self.idata[name]['desc']
		kp = self.cv_det.detect(gray)
		kp, desc = self.cv_desc.compute(gray, kp)
		matches = self.matcher.knnMatch(idesc, trainDescriptors=desc, k=2)
		p1, p2, kp_pairs = self.filter_matches(name, kp, matches)
		if len(p1) >= 4:
			H, status = cv2.findHomography(p1, p2, cv2.RANSAC, 2.0)
			dbprint('%d / %d  inliers/matched' % (np.sum(status), len(status)))
			pg = [p[0] for p in zip(p2, status) if p[1][0]]
			rs = np.average(pg, axis=0)
		return rs

	def find_image(self, name, frame=None):
		rs = None
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera)
#			frame = capture_cvimage(self.camera, resolution=(1280, 960))
		if not frame is None:
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			pnt = self._find_image(name, gray)
			if pnt is None:
				dbprint('tot shape=%s, %s' % (gray.shape, gray.dtype))
				n = 10
				dw = gray.shape[1] / n
				dh = gray.shape[0] / n
				for i in range(n-1):
					for j in range(n-1):
						x0 = i*dw
						y0 = j*dh
						x1 = (i+2)*dw
						y1 = (j+2)*dh
						f = gray[y0: y1, x0: x1]
						dbprint('%d,%d rect=%s shape=%s' % (i, j, (x0, y0, x1, y1), f.shape))
						pnt = self._find_image(name, f)
						if not pnt is None:
							break
					if not pnt is None:
						pnt[0] += x0
						pnt[1] += y0
						break
			if not pnt is None:
				cv2.circle(frame, (pnt[0], pnt[1]), max(20, frame.shape[0] / 20), (0, 0, 255), -1)
				rs = {
					'point': pnt,
					'frame': frame,
				}
		return rs

	def extract_black(self, frame):
		return frame

class ShapeSearch:

	def __init__(self, camera=None):
		self.camera = camera

	def polyArea(self, poly):
		return 0.5*np.abs(np.dot(poly[:,0],np.roll(poly[:,1],1))-np.dot(poly[:,1],np.roll(poly[:,0],1)))

	# threshold_type: THRESH_BINARY, THRESH_BINARY_INV, THRESH_TRUNC, THRESH_TOZERO, THRESH_TOZERO_INV
	def find_shapes(self, frame=None, threshold = 127, threshold_type=cv2.THRESH_BINARY, **params):
		rs = None
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera, **params)
#			frame = capture_cvimage(self.camera, resolution=(1280, 960))
		if not frame is None:
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			kernel = np.ones((5,5),np.float32)/25
			gray = cv2.filter2D(gray, -1, kernel)
			if threshold == 0:
#				thresh = cv2.Canny(gray, 0, 50, apertureSize=5)
#				thresh = cv2.dilate(thresh, None)
#				thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, threshold_type, 11, 2)
				thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, threshold_type, 11, 2)
			else:
				ret,thresh = cv2.threshold(gray, threshold, 255, threshold_type+cv2.THRESH_OTSU)
				ret,thresh = cv2.threshold(gray, ret, 255, threshold_type)
			contours,h = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			i = 0
			for cnt in contours:
				approx = cv2.approxPolyDP(cnt, 0.1*cv2.arcLength(cnt,True), True)
				prefix = 'found %d' % (len(approx),)
#				area = self.polyArea(approx[:, 0, :])
				area = cv2.contourArea(approx)
#				dbprint('cntarea=%s%s' % (cntarea, ' NOT SAME SIZE' if cntarea != area else ''))
				if area > 5 and area < (thresh.shape[0] * thresh.shape[1]) / 4:
					if len(approx)==5:
						dbprint("%s:pentagon: A:%s" % (prefix, area))
						cv2.drawContours(frame, [approx], 0, (255, 255 - i, 155), -1) # w
					elif len(approx)==3:
						dbprint("%s:triangle: A:%s" % (prefix, area))
						cv2.drawContours(frame, [approx], 0, (0, 255, i), -1) # green
					elif len(approx)==4:
						dbprint("%s:square: A:%s" % (prefix, area))
						cv2.drawContours(frame, [approx], 0, (i, 0, 255), -1) # blue
					elif len(approx) == 9:
						dbprint("%s:half-circle: A:%s" % (prefix, area))
						cv2.drawContours(frame, [approx], 0, (255, 255 - i, 0), -1) # red-green
					elif len(approx) > 15:
						dbprint("%s:circle: A:%s" % (prefix, area))
						cv2.drawContours(frame, [approx], 0, (0, 255, 255 - i), -1) # cyan (green-blue)
					else:
						i = i - 30
					i += 30
					if i > 100:
						i = 0
				else:
					dbprint('%s (too small)' % prefix)
			rs = {
				'contours': contours,
				'frame': frame,
				'thresh': thresh,
			}
		return rs

class ColorFix:

	def __init__(self, camera=None):
		self.camera = camera

	def max_rgb_filter(self, image):
		# split the image into its BGR components
		(B, G, R) = cv2.split(image)

		# find the maximum pixel intensity values for each
		# (x, y)-coordinate,, then set all pixel values less
		# than M to zero
		M = np.maximum(np.maximum(R, G), B)
		R[R < M] = 0
		G[G < M] = 0
		B[B < M] = 0

		# merge the channels back together and return the image
		return cv2.merge([B, G, R])

	def colorise(self, frame=None, **params):
		rs = None
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera, **params)
		if not frame is None:
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			equ = cv2.equalizeHist(gray)
			rs = {
				'frame': frame,
				'iframe': gray,
				'oframe': equ
			}
		return rs

	def colorise1(self, frame=None, **params):
		rs = None
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera, **params)
		if not frame is None:
			mframe = self.max_rgb_filter(frame)
			rs = {
				'frame': frame,
				'iframe': frame,
				'oframe': mframe
			}
		return rs

	def locate_object(self, lowertuple, highertuple, frame=None, **params):
		rs = None
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera, **params)
		if not frame is None:
			# resize the frame, blur it, and convert it to the HSV
			# color space
			blurred = cv2.GaussianBlur(frame, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
			# construct a mask, then perform
			# a series of dilations and erosions to remove any small
			# blobs left in the mask
			mask = cv2.inRange(hsv, np.array(lowertuple), np.array(highertuple))
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)
			# find contours in the mask and initialize the current
			# (x, y) center of the ball
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
			center = None
			# only proceed if at least one contour was found
			if len(cnts) > 0:
				dbprint('Found %d cnts' % len(cnts))
				# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and
				# centroid
				c = max(cnts, key=cv2.contourArea)
				((x, y), radius) = cv2.minEnclosingCircle(c)
				M = cv2.moments(c)
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				dbprint('Radius: %d' % radius)
				# only proceed if the radius meets a minimum size
				if radius > 10:
					oframe = frame.copy()
					# draw the circle and centroid on the frame,
					# then update the list of tracked points
					cv2.circle(oframe, (int(x), int(y)), int(radius),
						(0, 255, 255), 2)
					cv2.circle(oframe, center, 5, (0, 0, 255), -1)
					rs = {
						'center': center,
						'frame': frame,
						'iframe': blurred,
						'oframe': oframe
					}
		return rs

	def mask_range(self, lowertuple, highertuple, frame=None, **params):
		rs = None
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera, **params)
		if not frame is None:
			# resize the frame, blur it, and convert it to the HSV
			# color space
			blurred = cv2.GaussianBlur(frame, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
##			h,s,v = cv2.split(hsv)
			dw = 50
			dh = 50
			hlist = []
			iy = 0
			for y in range(0, hsv.shape[0], dh):
				ix = 0
				for x in range(0, hsv.shape[1], dw):
					hsvf = hsv[y:y+dh,x:x+dh]
#					hisf = cv2.calcHist([hsvf],[0], None, [20], [0, 256])
					hisf = cv2.calcHist([hsv], [0, 1], None, [18, 25], [0, 180, 0, 256])
##					nzl = np.nonzero(hisf)[0]
#					nzl = np.where(hisf > 10)[0]
#					hlist.append({'x': x, 'y': y, 'w': hsvf.shape[1], 'h': hsvf.shape[0], 'hist': list(nzl)})
					hlist.append({'x': x, 'y': y, 'w': hsvf.shape[1], 'h': hsvf.shape[0], 'hist': hisf, 'img': blurred[y:y+dh,x:x+dh], 'ix': ix, 'iy': iy})
					ix += 1
				iy += 1
			# construct a mask, then perform
			# a series of dilations and erosions to remove any small
			# blobs left in the mask
			mask = cv2.inRange(hsv, np.array(lowertuple), np.array(highertuple))
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)
			oframe = cv2.bitwise_and(blurred, blurred, mask = mask)
			rs = {
				'hlist': hlist,
				'frame': frame,
				'iframe': blurred,
				'oframe': oframe
			}
		return rs
