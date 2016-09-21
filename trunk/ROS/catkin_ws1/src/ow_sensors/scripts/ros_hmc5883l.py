#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy
from hmc5883l import hmc5883l
from std_msgs.msg import Float32

if __name__ == '__main__':

	try:
		last_heading = None
		hmc = hmc5883l(gauss = 4.7, declination = (-2,5))
		rospy.loginfo("HMC5883l on address 0x%x" % (hmc.address))
		rospy.init_node('hmc5883l', anonymous = True)
		pub = rospy.Publisher('/sensors/hmc5883l', Float32, queue_size=5)
		while not rospy.is_shutdown():
			heading = hmc.heading()
			if heading != last_heading:
				pub.publish(Float32(heading))
				last_heading = heading
			rospy.sleep(0.01)
	except rospy.ROSInterruptException:
		pass