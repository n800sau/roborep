#!/usr/bin/env python

from __future__ import print_function
import roslib

import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class motion_detector(rospy.SubscribeListener):

	def __init__(self):

		self.thresh_pub = rospy.Publisher("/thresh/image_raw", Image, queue_size=1, subscriber_listener=self)
		self.motion_pub = rospy.Publisher("/motion/image_raw", Image, queue_size=1, subscriber_listener=self)

		self.peers = []

		self.frames = []
		self.avg = None

		self.min_area = rospy.get_param('~min_area', 5000)
		self.min_aspect = rospy.get_param('~min_aspect', 1/2.)
		self.max_aspect = rospy.get_param('~max_aspect', 1/0.5)
		self.max_width = rospy.get_param('~max_width', 100)
		self.max_height = rospy.get_param('~max_height', 100)
		self.delta_thresh = rospy.get_param('~delta_thresh', 5)

		self.bridge = CvBridge()
		self.image_sub = None

	def peer_subscribe(self, topic_name, topic_publish, peer_publish):
		if self.image_sub is None:
			self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
			rospy.loginfo('Subscribe')
			self.peers = 1
		else:
			self.peers += 1
		rospy.loginfo('N peers: %d' % self.peers)

	def peer_unsubscribe(self, topic_name, num_peers):
		if not self.image_sub is None:
			self.peers -= 1
		if self.peers == 0:
			self.image_sub.unregister()
			self.image_sub = None
			rospy.loginfo('Unsubscribe')
		rospy.loginfo('N peers: %d' % self.peers)

	def callback(self, data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		self.frames.append(frame)
		self.frames = self.frames[-2:]

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)

		if self.avg is None:
			# the initial frame
			rospy.loginfo("Starting background model...")

			self.avg = gray.copy().astype("float")
		else:

			text = "Unoccupied"

			# accumulate the weighted average between the current frame and
			# previous frames, then compute the difference between the current
			# frame and running average
			cv2.accumulateWeighted(gray, self.avg, 0.5)
			frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg))

			# threshold the delta image, dilate the thresholded image to fill
			# in holes, then find contours on thresholded image
			thresh = cv2.threshold(frameDelta, self.delta_thresh, 255, cv2.THRESH_BINARY)[1]

#			thresh = cv2.adaptiveThreshold(frameDelta, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
			thresh = cv2.dilate(thresh, None, iterations=2)
			(cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

			ts = time.strftime("%A %d %B %Y %I:%M:%S%p", time.localtime(data.header.stamp.secs))

			occupied = False
#			rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~min_area'), self.min_area)

			good_cnts = []
#			rospy.logdebug('Total %d cnts' % len(cnts))
			# loop over the contours
			for c in cnts:

				area = cv2.contourArea(c)
#				rospy.logdebug('Area %s' % area)
				# if the contour is too small, ignore it
				if area < self.min_area:
					continue

				# compute the bounding box for the contour, draw it on the frame,
				# and update the text
				(x, y, w, h) = cv2.boundingRect(c)

				if w > self.max_width or h > self.max_height:
					continue

				# check aspect
				aspect = w/float(h)
				if aspect < self.min_aspect or aspect > self.max_aspect:
					continue

				good_cnts.append({'c':c, 'x': x, 'y': y, 'w': w, 'h': h, 'area': area})

				fimg = frame[y:y+h, x:x+w]
#	#	#		gray = cv2.cvtColor(fimg, cv2.COLOR_BGR2GRAY)
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
				text = "Occupied"
				occupied = True

			if good_cnts:
				rospy.logdebug('Found at %d cnts' % len(good_cnts))

			for c in good_cnts:
				rospy.logdebug('\t%d,%d %dx%d area=%d' % (c['x'], c['y'], c['w'], c['h'], c['area']))

			# draw the text and timestamp on the frame
			cv2.putText(frame, "Room Status: {}".format(text), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
			cv2.putText(frame, ts, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

			self.thresh_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "mono8"))

			if occupied:
				self.motion_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main(args):
	rospy.init_node('motion_detector', log_level=rospy.DEBUG)
	ic = motion_detector()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
		main(sys.argv)
