#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, math
import rospy
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
from ow_sensors.msg import HeadingStamped

class IMUcalc:

	def __init__(self):
		self.last_acc = None
		self.last_gyro = None
		self.last_ts = None
		self.angle = 0
		# Ordered this way to minimize wait time.
		rospy.init_node('imu', anonymous = True)
		self.r = rospy.Rate(10)
		rospy.Subscriber('/sensors/l3g4200/axes', Vector3Stamped, self.on_gyro)
		rospy.Subscriber('/sensors/adxl345/axes', Vector3Stamped, self.on_acc)
		self.pub_angle = rospy.Publisher('/sensors/imu/angle', HeadingStamped, queue_size=1)

	def run(self):
		rospy.spin()

	def publish_imu(self):
		if self.pub_angle.get_num_connections() > 0:
			if not (self.last_acc is None or self.last_gyro is None):
				last_acc_ts = self.last_acc.header.stamp.secs + self.last_acc.header.stamp.nsecs / 1000000000.
				last_gyro_ts = self.last_gyro.header.stamp.secs + self.last_gyro.header.stamp.nsecs / 1000000000.
				if self.last_ts is None:
					self.last_ts = max(last_acc_ts, last_gyro_ts)
				else:
					if abs(last_acc_ts - last_gyro_ts) < 0.1:
						acc_angle = math.atan2(self.last_acc.vector.y, self.last_acc.vector.x)
						gyro_angle = math.atan2(self.last_gyro.vector.y, self.last_gyro.vector.x)
						dt = max(last_acc_ts, last_gyro_ts) - self.last_ts
						angle = 0.98 * (self.angle + gyro_angle*dt) + 0.02 * acc_angle
						ts = rospy.Time.now()
						self.pub_angle.publish(header=Header(stamp=ts), heading=angle)

	def on_gyro(self, data):
		self.last_gyro = data
		self.publish_imu()

	def on_acc(self, data):
		self.last_acc = data
		self.publish_imu()

if __name__ == '__main__':


	node = IMUcalc()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
