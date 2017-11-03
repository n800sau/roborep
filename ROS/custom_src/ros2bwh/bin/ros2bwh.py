#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, random, time
import rospy
import serial
from geometry_msgs.msg import Quaternion, Twist, Pose
from tf.broadcaster import TransformBroadcaster

DEFPORT = '/dev/ttyS1'
# max timeout of no data before debug message is emitted
MAX_DATA_TIMEOUT = 10

def str2float(datalist):
	return [float(v) for v in datalist]

class BlueWheel:

	def __init__(self):
		serport = rospy.get_param('bwh_serial_port', DEFPORT)
		self.ser = serial.Serial(serport, 115200)

	def cmdVelCallback(self, req):
		# Handle velocity-based movement requests
		self.last_cmd_vel = rospy.Time.now()

		x = req.linear.x		 # m/s
		th = req.angular.z	   # rad/s

		if x == 0:
			# Turn in place
			right = th * self.wheel_track  * self.gear_reduction / 2.0
			left = -right
		elif th == 0:
			# Pure forward/backward motion
			left = right = x
		else:
			# Rotation about a point in space
			left = x - th * self.wheel_track  * self.gear_reduction / 2.0
			right = x + th * self.wheel_track  * self.gear_reduction / 2.0
		if left < -1:
			left = -1
		if left > 1:
			left = 1
		if right < -1:
			right = -1
		if right > 1:
			right = 1
		left = int(127 * left)
		right = int(127 * right)
#		rospy.loginfo("wt:%.2f, gr:%.2f" % (self.wheel_track, self.gear_reduction))
		rospy.loginfo("x:%.2f, th:%.2f %.2f <> %.2f" % (x, th, left, right))
		self.ser.write("set m1vel %d\n" % (left,))
		self.ser.write("set m2vel %d\n" % (right,))
		self.ser.flush()
		reply = self.ser.read()
		rospy.loginfo('reply: %s' % reply)

	def run(self):
		rospy.init_node('blue_wheel', anonymous = True)
		# Subscriptions
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
		# Publications
		rospy.spin()

if __name__ == '__main__':

	node = BlueWheel()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
