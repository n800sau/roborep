import os
from picamera.array import PiRGBArray
import cv2
from utils import dbprint
import numpy as np

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
DETECTOR = 'GFTT'

# SIFT/0, SURF*, BRIEF+, BRISK*, ORB+, FREAK*
EXTRACTOR = 'BRIEF'


MATCHER = 'BruteForce-Hamming'

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
					rs = (lp * 100) / self.kpl
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

	def find_image(self, name, frame=None):
		rs = None
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera)
#			frame = capture_cvimage(self.camera, resolution=(1280, 960))
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		if not frame is None:
			kp = self.cv_det.detect(gray)
			kp, desc = self.cv_desc.compute(gray, kp)
			idesc = self.idata[name]['desc']
			matches = self.matcher.knnMatch(idesc, trainDescriptors=desc, k=2)
			p1, p2, kp_pairs = self.filter_matches(name, kp, matches)
			if len(p1) >= 4:
				H, status = cv2.findHomography(p1, p2, cv2.RANSAC, 3.0)
				dbprint('%d / %d  inliers/matched' % (np.sum(status), len(status)))
				pg = [p[0] for p in zip(p2, status) if p[1][0]]
				pnt = np.average(pg, axis=0)
				cv2.circle(frame, (pnt[0], pnt[1]), 20, (0, 0, 255), -1)
				rs = {
					'point': pnt,
					'frame': frame,
				}
		return rs
