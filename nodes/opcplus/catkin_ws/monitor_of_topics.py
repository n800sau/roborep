#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, random, time
import rospy
from std_msgs.msg import Int16, Bool
from sensor_msgs.msg import JointState

class Monitor:

	def __init__(self):
		self.vcount = 7
		self.ns = '/opcplus'
		rospy.loginfo('monitor is ready')
		self.reset()

	def reset(self):
		self.csvdata = [''] * self.vcount
		self.csvcount = [0] * self.vcount

	def data_cb(self, req, args):
		pos = args[0]
		self.csvdata[pos] = req.data
		self.csvcount[pos] += 1

	def run(self):
		rospy.init_node('wheel_monitor', anonymous = True)
		# Subscriptions
		rospy.Subscriber(self.ns + "/lwheel_desired_rate", Int16, callback=self.data_cb, callback_args = [0])
		rospy.Subscriber(self.ns + "/lwheel_pwr", Int16, callback=self.data_cb, callback_args = [1])
		rospy.Subscriber(self.ns + "/lwheel_rate", Int16, callback=self.data_cb, callback_args = [2])
		rospy.Subscriber(self.ns + "/rwheel_desired_rate", Int16, callback=self.data_cb, callback_args = [4])
		rospy.Subscriber(self.ns + "/rwheel_pwr", Int16, callback=self.data_cb, callback_args = [5])
		rospy.Subscriber(self.ns + "/rwheel_rate", Int16, callback=self.data_cb, callback_args = [6])

		print '|'.join(['{:>16s}'.format(v) for v in ['Lreq', 'Lpwr', 'Lrt', '', 'Rreq', 'Rpwr', 'Rrt']])
		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if len([v for v in self.csvdata if v == '']) < self.vcount:
				print '|'.join([('{:16d}'.format(v/self.csvcount[i]) if v != '' else (' ' * 16)) for i,v in enumerate(self.csvdata)])
				self.reset()
			r.sleep()

if __name__ == '__main__':

	node = Monitor()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
