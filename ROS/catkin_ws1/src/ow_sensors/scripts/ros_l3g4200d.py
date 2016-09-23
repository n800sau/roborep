#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy
from l3g4200d import l3g4200
from geometry_msgs.msg import Vector3Stamped, Vector3
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header

if __name__ == '__main__':

	try:
		gyro = l3g4200(1)
		rospy.loginfo("L3G4200 on address 0x%x" % (gyro.address))
		# Ordered this way to minimize wait time.
		rospy.init_node('l3g4200', anonymous = True)
		r = rospy.Rate(10)
		pub_axes = rospy.Publisher('/sensors/l3g4200/axes', Vector3Stamped, queue_size=1)
		pub_t = rospy.Publisher('/sensors/l3g4200/t', Temperature, queue_size=3)
		while not rospy.is_shutdown():
			if pub_t.get_num_connections() > 0 or pub_axes.get_num_connections() > 0:
				st = gyro.bus.read_byte_data(gyro.address, gyro.STATUS_REG)
				if st:
					axes = dict(zip(['x', 'y', 'z'], gyro.getDegPerSecAxes()))
					t = gyro.getDieTemperature()
					ts = rospy.Time.now()
					pub_axes.publish(header=Header(stamp=ts), vector=Vector3(**axes))
					pub_t.publish(header=Header(stamp=ts), temperature=t)
			r.sleep()
	except rospy.ROSInterruptException:
		pass
