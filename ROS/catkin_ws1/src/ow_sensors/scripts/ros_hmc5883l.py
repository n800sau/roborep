#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy
from hmc5883l import hmc5883l
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
from ow_sensors.msg import HeadingStamped

if __name__ == '__main__':

	try:
		hmc = hmc5883l(gauss = 4.7, declination = (-2,5))
		rospy.loginfo("HMC5883l on address 0x%x" % (hmc.address))
		rospy.init_node('hmc5883l', anonymous = True)
		r = rospy.Rate(10)
		pub = rospy.Publisher('/sensors/hmc5883l/axes', Vector3Stamped, queue_size=1)
		pub_h = rospy.Publisher('/sensors/hmc5883l/heading',HeadingStamped, queue_size=1)
		while not rospy.is_shutdown():
			if pub.get_num_connections() > 0 or pub_h.get_num_connections():
				axes = hmc.axes()
				heading = hmc.heading(axes=axes)
				ts = rospy.Time.now()
				axes = dict(zip(['x', 'y', 'z'], axes))
				pub.publish(header=Header(stamp=ts), vector=Vector3(**axes))
				pub_h.publish(header=Header(stamp=ts), heading=heading)
			r.sleep()
	except rospy.ROSInterruptException:
		pass
