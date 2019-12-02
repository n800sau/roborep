#!/usr/bin/env python

"""
	A base controller class for the Arduino microcontroller

	Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

	Created for the Pi Robot Project: http://www.pirobot.org
	Copyright (c) 2010 Patrick Goebel.  All rights reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details at:

	http://www.gnu.org/licenses
"""
import roslib
import rospy
import os

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from tf.broadcaster import TransformBroadcaster

""" Class to receive Twist commands"""
class BaseController:
	def __init__(self, arduino, name="base_controllers"):
		self.arduino = arduino
		self.name = name
		self.rate = float(rospy.get_param("~base_controller_rate", 10))
		self.stopped = False

		now = rospy.Time.now()
		self.then = now # time for determining dx/dy
		self.t_delta = rospy.Duration(1.0 / self.rate)
		self.t_next = now + self.t_delta

		self.wheel_track = rospy.get_param("~wheel_track", 0.4)
		self.gear_reduction = rospy.get_param("~gear_reduction", 1.0)

		# Subscriptions
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

	def stop(self):
		self.stopped = True
		self.arduino.do_stop()

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
		self.arduino.do_move("%d %d" % (left, right))
