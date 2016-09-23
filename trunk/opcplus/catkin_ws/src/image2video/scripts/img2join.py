#!/usr/bin/env python

import sys, time
# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage, Image
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError
from resultsmontage import ResultsMontage

VERBOSE=False

class image2join:

	def __init__(self):
		'''Initialize ros publisher, ros subscriber'''
		self.ow_img = None
		self.uvc_img = None

		self.image_pub = rospy.Publisher("/duplex/image_raw", Image, queue_size=1)
		self.image_pub_jpg = rospy.Publisher("/duplex/image_raw/compressed", CompressedImage, queue_size=1)
		self.image_pub_h = rospy.Publisher("/duplex_h/image_raw", Image, queue_size=1)
		self.image_pub_jpg_h = rospy.Publisher("/duplex_h/image_raw/compressed", CompressedImage, queue_size=1)

		# subscribed Topic
		self.ow_sub = rospy.Subscriber("/ow_camera/image_raw/compressed", CompressedImage, self.callback_ow, queue_size = 1)
		self.uvc_sub = rospy.Subscriber("/image_raw/compressed", CompressedImage, self.callback_uvc, queue_size = 1)
		if VERBOSE :
			print "subscribed to /camera/image/compressed"

	def callback_ow(self, ros_data):
		self.ow_img = self.data2img(ros_data)
		if not self.uvc_img is None:
			self.make_frame()

	def callback_uvc(self, ros_data):
		self.uvc_img = self.data2img(ros_data)
		if not self.ow_img is None:
			self.make_frame()

	def data2img(self, ros_data):
		if VERBOSE :
			print 'received image of type: "%s"' % ros_data.format

		#### direct conversion to CV2 ####
		np_arr = np.fromstring(ros_data.data, np.uint8)
		return cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

	def make_frame(self):
		#### Create Image ####
		sz = (160, 120)
		img_left = cv2.resize(self.ow_img, sz)
		img_right = cv2.resize(self.uvc_img, sz)
		montage = ResultsMontage((sz[1], sz[0]), 1, 2)
		montage.addResult(img_left)
		montage.addResult(img_right)
		msg = Image()
		msg.header.stamp = rospy.Time.now()
		msg.encoding = 'bgr8'
		msg.step = montage.montage.shape[1] * 3
		msg.height = montage.montage.shape[0]
		msg.width = montage.montage.shape[1]
		msg.data = np.array(montage.montage).tostring()
		# Publish new image
		self.image_pub.publish(msg)
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = 'jpeg'
		msg.data = np.array(cv2.imencode('.jpg', montage.montage)[1]).tostring()
		self.image_pub_jpg.publish(msg)
		montage = ResultsMontage((sz[1], sz[0]), 2, 2)
		montage.addResult(img_left)
		montage.addResult(img_right)
		msg = Image()
		msg.header.stamp = rospy.Time.now()
		msg.encoding = 'bgr8'
		msg.step = montage.montage.shape[1] * 3
		msg.height = montage.montage.shape[0]
		msg.width = montage.montage.shape[1]
		msg.data = np.array(montage.montage).tostring()
		# Publish new image
		self.image_pub_h.publish(msg)
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = 'jpeg'
		msg.data = np.array(cv2.imencode('.jpg', montage.montage)[1]).tostring()
		self.image_pub_jpg_h.publish(msg)

		self.ow_img = None
		self.uvc_img = None

def main(args):
	'''Initializes and cleanup ros node'''
	ic = image2join()
	rospy.init_node('image2join', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS Image feature detector module"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
