#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy
from adxl345 import ADXL345
from geometry_msgs.msg import Vector3

if __name__ == '__main__':

	try:
		adxl345_address = 0x1d
		adxl345 = ADXL345(adxl345_address)
		rospy.loginfo("ADXL345 on address 0x%x" % (adxl345.address))
		# Ordered this way to minimize wait time.
		adxl345.setFIFOmode(adxl345.FIFO_BYPASS, adxl345.FIFO_TRIGGER_INT2, 2)
		adxl345.setFIFOmode(adxl345.FIFO_FIFO, adxl345.FIFO_TRIGGER_INT2, 2)
		rospy.init_node('adxl345', anonymous = True)
		pub = rospy.Publisher('/sensors/adxl345', Vector3, queue_size=5)
		while not rospy.is_shutdown():
			axes_list = adxl345.getAxesList()
			if axes_list:
				axes = axes_list[-1]
				pub.publish(Vector3(axes['x'], axes['y'], axes['z']))
			rospy.sleep(0.01)
	except rospy.ROSInterruptException:
		pass