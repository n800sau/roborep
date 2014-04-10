#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy, cv2, cv_bridge, numpy
from time import time, sleep
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

def callback(data):
#	try:
	bridge = cv_bridge.CvBridge()
	cv_image = bridge.imgmsg_to_cv(data, "mono8")
	data = numpy.array(cv_image)
	dataval = data.mean() / 256.
#		print cv2.calcHist( [data], [0], None, [8], [0, 255] )
#		print data.mean(), data.max(), data.dtype
#		ret, mask = cv2.threshold(data, 0, 100, cv2.THRESH_BINARY)
#		print cv2.mean(cv_image)
	pub.publish(dataval)
#	except cv_bridge.CvBridgeError, e:
#		rospy.logerr(e)



rospy.init_node('brightness', anonymous = True)

rospy.Subscriber('/camera/rgb/image_raw', Image, callback)
pub = rospy.Publisher('/brightness', Float32)

rospy.spin()
