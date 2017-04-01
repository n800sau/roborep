#!/usr/bin/env python

import rospy
from oculus_remnant.base_controller import BaseController
from geometry_msgs.msg import Twist
import os, time
import thread
from serial.serialutil import SerialException
from oculus_remnant.oculus_intf import OculusIntf

class OculusBaseNode():
	def __init__(self):
		rospy.init_node('oculus_base_node', log_level=rospy.INFO)

		# Get the actual node name in case it is set in the launch file
		self.name = rospy.get_name()

		# Cleanup when termniating the node
		rospy.on_shutdown(self.shutdown)

		self.base_frame = rospy.get_param("~base_frame", 'base_link')

		# Overall loop rate: should be faster than fastest sensor rate
		self.rate = int(rospy.get_param("~rate", 10))
		r = rospy.Rate(self.rate)

		# Initialize a Twist message
		self.cmd_vel = Twist()

		# A cmd_vel publisher so we can stop the robot when shutting down
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

		# Initialize the controlller
		self.controller = OculusIntf()

		# Initialize the base controller if used
		self.myBaseController = BaseController(self.controller, self.name + "_base_controller")

		# Start polling the sensors and base controller
		while not rospy.is_shutdown():

			r.sleep()

	def shutdown(self):
		rospy.loginfo("Shutting down Arduino Node...")

		# Stop the robot
		try:
			rospy.loginfo("Stopping the robot...")
			self.cmd_vel_pub.Publish(Twist())
			rospy.sleep(2)
		except:
			pass

		# Close the serial port
		try:
			self.controller.close()
		except:
			pass
		finally:
			rospy.loginfo("Serial port closed.")
			os._exit(0)

if __name__ == '__main__':
	try:
		OculusBaseNode()
	except SerialException:
		rospy.logerr("Serial exception trying to open port.")
		os._exit(0)
