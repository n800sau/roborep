import os
from picamera.array import PiRGBArray
import cv2
from utils import dbprint
import numpy as np

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
DETECTOR = 'GridORB'
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
#		self.matcher = cv2.DescriptorMatcher_create(MATCHER)
		#flann_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		flann_params= dict(algorithm = FLANN_INDEX_LSH,
			table_number = 6, # 12
			key_size = 12,     # 20
			multi_probe_level = 1) #2
		self.matcher = cv2.FlannBasedMatcher(flann_params, {})  # bug : need to pass empty dict (#1329)


	def filterMatches(self, kp, matches, ratio = 0.75):
		mkp1, mkp2 = [], []
		for m in matches:
			if len(m) == 2 and m[0].distance < m[1].distance * ratio:
				m = m[0]
				mkp1.append(self.kp[m.queryIdx])
				mkp2.append(kp[m.trainIdx])
		return zip( mkp1, mkp2 )

	def percent(self, frame=None):
		rs = None
		try:
			if frame is None and self.camera:
				frame = capture_cvimage(self.camera)
			if not frame is None:
#			frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
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
					pairs = self.filterMatches(kp, matches)
					lp = len(pairs)
					rs = ((lp * 100) / self.kpl) if self.kpl else 0
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

	def find_shapes(self, frame=None):
		rs = None
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera)
#			frame = capture_cvimage(self.camera, resolution=(1280, 960))
		if not frame is None:
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			ret,thresh = cv2.threshold(gray, 127, 255, 1)
			contours,h = cv2.findContours(thresh, 1, 2)
			for cnt in contours:
				approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True), True)
				prefix = 'found %d ' % len(approx)
				if len(approx)==5:
					dbprint(prefix + ":pentagon")
					cv2.drawContours(frame, [cnt], 0, 255, -1) # w
				elif len(approx)==3:
					dbprint(prefix + ":triangle")
					cv2.drawContours(frame, [cnt], 0, (0, 255, 0), -1) # green
				elif len(approx)==4:
					dbprint(prefix + ":square")
					cv2.drawContours(frame, [cnt], 0, (0, 0, 255), -1) # blue
				elif len(approx) == 9:
					dbprint(prefix + ":half-circle")
					cv2.drawContours(frame, [cnt], 0, (255, 255, 0), -1) # red-green
				elif len(approx) > 15:
					dbprint(prefix + ":circle")
					cv2.drawContours(frame, [cnt], 0, (0, 255, 255), -1) # cyan (green-blue)
				dbprint(prefix)
			rs = {
				'contours': contours,
				'frame': frame,
			}
		return rs
