#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy
from time import time, sleep
from l3g4200d import l3g4200
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

if __name__ == '__main__':

	try:
		gyro = l3g4200(1)
		rospy.loginfo("L3G4200 on address 0x%x" % (gyro.address))
		# Ordered this way to minimize wait time.
		rospy.init_node('l3g4200', anonymous = True)
		pub_degsec = rospy.Publisher('/sensors/l3g4200/degsec', Vector3, queue_size=3)
		pub_t = rospy.Publisher('/sensors/l3g4200/t', Float32, queue_size=3)
		while not rospy.is_shutdown():
			st = gyro.bus.read_byte_data(gyro.address, gyro.STATUS_REG)
			if st:
				degsec = gyro.getDegPerSecAxes()
				t = gyro.getDieTemperature()
				pub_degsec.publish(Vector3(*degsec))
				pub_t.publish(Float32(t))
			rospy.sleep(0.01)
	except rospy.ROSInterruptException:
		pass