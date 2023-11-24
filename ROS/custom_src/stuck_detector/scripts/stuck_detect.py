#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, random
import rospy
from opencv_apps.msg import Point2DStamped
from std_msgs.msg import Header
from stuck_detector.msg import Stuck
from nav_msgs.msg import Odometry

class StuckDetect:

	def __init__(self):
		self.stopped_ts = None

	def run(self):
		rospy.init_node('stuck_detector', anonymous = True)
		self.subs_phase_corr = rospy.Subscriber("/phase_corr/shift", Point2DStamped, self.on_shift)
		self.subs_odom = rospy.Subscriber("/fchassis/odom", Odometry, self.on_odom)
		self.pub_stuck = rospy.Publisher('/fchassis/stuck', Stuck, queue_size=1)
		rospy.spin()

	def on_shift(self, shift):
		stopped = abs(shift.point.x) < 0.5
		if stopped:
			if self.stopped_ts is None:
				self.stopped_ts = shift.header.stamp
		else:
			self.stopped_ts = None

	def on_odom(self, odom):
		if odom.twist.twist.linear.x > 0 or odom.twist.twist.linear.y > 0 or odom.twist.twist.angular.z > 0:
			stuck = False
			if not self.stopped_ts is None:
				tdiff = odom.header.stamp - self.stopped_ts
				stuck = tdiff > rospy.Duration(secs=2)
			self.pub_stuck.publish(header=Header(stamp=odom.header.stamp, frame_id='odom'), stuck=stuck)
		else:
			self.stopped_ts = None

if __name__ == '__main__':


	node = StuckDetect()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
