import os
from picamera.array import PiRGBArray
import cv2
from utils import dbprint

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

def set_params(camera):
	camera.resolution = (320, 240)
	camera.exposure_mode = 'auto'
	camera.brightness = 70
	camera.contrast = 70

def update_img(camera, fname=None):
	fname = os.path.join(os.path.expanduser('~/public_html'), fname or 'picam_0.jpg')
	set_params(camera)
	camera.capture(fname, use_video_port=False)

def capture_cvimage(camera):
	# initialize the camera and grab a reference to the raw camera capture
	stream = PiRGBArray(camera)
	set_params(camera)
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
		if frame is None and self.camera:
			frame = capture_cvimage(self.camera)
		if not frame is None:
#			frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
			if self.img is None:
				self.img = frame
				self.kp = self.cv_det.detect(self.img)
				self.kp, self.desc = self.cv_desc.compute(self.img, self.kp)
				self.kpl = len(self.kp)
			else:
				kp = self.cv_det.detect(frame)
				kp, desc = self.cv_desc.compute(frame, kp)
				matches = self.matcher.knnMatch(self.desc, trainDescriptors=desc, k=2)
				pairs = self.filterMatches(kp, matches)
				lp = len(pairs)
				rs = (lp * 100) / self.kpl
		return rs
