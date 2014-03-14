#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def init_feature(name):
	chunks = name.split('-')
	if chunks[0] == 'sift':
		detector = cv2.SIFT()
		norm = cv2.NORM_L2
	elif chunks[0] == 'surf':
		detector = cv2.SURF(800)
		norm = cv2.NORM_L2
	elif chunks[0] == 'orb':
		detector = cv2.ORB(400)
		norm = cv2.NORM_HAMMING
	else:
		return None, None
	if 'flann' in chunks:
		if norm == cv2.NORM_L2:
			flann_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		else:
			flann_params= dict(algorithm = FLANN_INDEX_LSH,
							   table_number = 6, # 12
							   key_size = 12,	  # 20
							   multi_probe_level = 1) #2
		matcher = cv2.FlannBasedMatcher(flann_params, {})  # bug : need to pass empty dict (#1329)
	else:
		matcher = cv2.BFMatcher(norm)
	return detector, matcher

def filter_matches(kp1, kp2, matches, ratio = 0.75):
	mkp1, mkp2 = [], []
	for m in matches:
		if len(m) == 2 and m[0].distance < m[1].distance * ratio:
			m = m[0]
			mkp1.append( kp1[m.queryIdx] )
			mkp2.append( kp2[m.trainIdx] )
	p1 = np.float32([kp.pt for kp in mkp1])
	p2 = np.float32([kp.pt for kp in mkp2])
	kp_pairs = zip(mkp1, mkp2)
	return p1, p2, kp_pairs

def explore_match(img1, img2, kp_pairs, status = None, H = None):
#def explore_match(win, img1, img2, kp_pairs, status = None, H = None):
	h1, w1 = img1.shape[:2]
	h2, w2 = img2.shape[:2]
#	print '###########', img1.shape, img2.shape
#	print '###########', img1.dtype, img2.dtype
	vis = np.zeros((max(h1, h2), w1+w2, 3), np.uint8)
	vis[:h1, :w1] = img1
	vis[:h2, w1:w1+w2] = img2
#	vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

	if H is not None:
		corners = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]])
		corners = np.int32( cv2.perspectiveTransform(corners.reshape(1, -1, 2), H).reshape(-1, 2) + (w1, 0) )
		cv2.polylines(vis, [corners], True, (255, 255, 255))

	if status is None:
		status = np.ones(len(kp_pairs), np.bool_)
	p1 = np.int32([kpp[0].pt for kpp in kp_pairs])
	p2 = np.int32([kpp[1].pt for kpp in kp_pairs]) + (w1, 0)

	green = (0, 255, 0)
	red = (0, 0, 255)
	white = (255, 255, 255)
	kp_color = (51, 103, 236)
	for (x1, y1), (x2, y2), inlier in zip(p1, p2, status):
		if inlier:
			col = green
			cv2.circle(vis, (x1, y1), 2, col, -1)
			cv2.circle(vis, (x2, y2), 2, col, -1)
		else:
			col = red
			r = 2
			thickness = 3
			cv2.line(vis, (x1-r, y1-r), (x1+r, y1+r), col, thickness)
			cv2.line(vis, (x1-r, y1+r), (x1+r, y1-r), col, thickness)
			cv2.line(vis, (x2-r, y2-r), (x2+r, y2+r), col, thickness)
			cv2.line(vis, (x2-r, y2+r), (x2+r, y2-r), col, thickness)
	vis0 = vis.copy()
	for (x1, y1), (x2, y2), inlier in zip(p1, p2, status):
		if inlier:
			cv2.line(vis, (x1, y1), (x2, y2), green)

#	cv2.imshow(win, vis)
#	def onmouse(event, x, y, flags, param):
#		cur_vis = vis
#		if flags & cv2.EVENT_FLAG_LBUTTON:
#			cur_vis = vis0.copy()
#			r = 8
#			m = (anorm(p1 - (x, y)) < r) | (anorm(p2 - (x, y)) < r)
#			idxs = np.where(m)[0]
#			kp1s, kp2s = [], []
#			for i in idxs:
#				 (x1, y1), (x2, y2) = p1[i], p2[i]
#				 col = (red, green)[status[i]]
#				 cv2.line(cur_vis, (x1, y1), (x2, y2), col)
#				 kp1, kp2 = kp_pairs[i]
#				 kp1s.append(kp1)
#				 kp2s.append(kp2)
#			cur_vis = cv2.drawKeypoints(cur_vis, kp1s, flags=4, color=kp_color)
#			cur_vis[:,w1:] = cv2.drawKeypoints(cur_vis[:,w1:], kp2s, flags=4, color=kp_color)

#		cv2.imshow(win, cur_vis)
#	cv2.setMouseCallback(win, onmouse)
	return vis


class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher('/image_circle',Image)

#		self.win = "Image window"
#		cv.NamedWindow(self.win, 1)
#		cv.MoveWindow(self.win, 25, 800)
#		cv.ResizeWindow(self.win, 160, 120)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.callback)
		feature_name = 'surf'
		self.detector, self.matcher = init_feature(feature_name)
		if self.detector != None:
			print 'using', feature_name
		else:
			print 'unknown feature:', feature_name
			sys.exit(1)

		self.eyeimg = cv2.imread('eye1.jpg', cv.CV_LOAD_IMAGE_COLOR)
		self.eye_kp, self.eye_desc = self.detector.detectAndCompute(self.eyeimg, None)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
		except CvBridgeError, e:
			print e

#		cv2.imshow("eyeimg", self.eyeimg)
#		cv.ShowImage("cv_image", cv_image)
		cv_image = np.array(cv_image)
#		cv2.cvtColor(cv_image, cv.CV_BGR2GRAY)

		kp, desc = self.detector.detectAndCompute(cv_image, None)
		print 'eye - %d features, frame - %d features' % (len(self.eye_kp), len(kp))

#		def match_and_draw(win):
		def match_and_draw():
			print 'matching...'
			raw_matches = self.matcher.knnMatch(self.eye_desc, trainDescriptors = desc, k = 2) #2
			p1, p2, kp_pairs = filter_matches(self.eye_kp, kp, raw_matches)
			print 'found %d' % len(p1)
			if len(p1) >= 4:
				H, status = cv2.findHomography(p1, p2, cv2.RANSAC, 5.0)
				print '%d / %d	inliers/matched' % (np.sum(status), len(status))
				vis = explore_match(self.eyeimg, cv_image, kp_pairs, status, H)

				new_image = self.convert_image(vis)
				try:
					self.image_pub.publish(self.bridge.cv_to_imgmsg(new_image, "bgr8"))		
				except CvBridgeError, e:
					print e

#				vis = explore_match(win, self.eyeimg, cv_image, kp_pairs, status, H)
#				while True:
#					c = cv.WaitKey(7) % 0x100
#					if c == 27:
#						break
#				print 'Continue...'
#			else:
#				H, status = None, None
				print '%d matches found, not enough for homography estimation' % len(p1)


		match_and_draw()
#		match_and_draw(self.win)


#		try:
#			self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
#		except CvBridgeError, e:
#			print e

def main(args):
	rospy.init_node('image_converter', anonymous=True)
	ic = image_converter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv.DestroyAllWindows()

if __name__ == '__main__':
		main(sys.argv)

