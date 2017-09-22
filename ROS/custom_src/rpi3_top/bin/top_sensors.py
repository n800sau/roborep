#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, random
import rospy
import serial
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

IMU_LINK = 'imubox_link'
DEFPORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0'

def str2float(datalist):
	return [float(v) for v in datalist]

class TopSensors:

	def __init__(self):
		serport = rospy.get_param('sensors_serial_port', DEFPORT)
		self.ser = serial.Serial(serport, 115200)

	def run(self):
		rospy.init_node('top_sensors', anonymous = True)
		self.pub_imu = rospy.Publisher('imu', Imu, queue_size=1)
		data = {}
		while not rospy.is_shutdown():
			line = self.ser.readline().strip()
			if line:
				line = line.split()
				if line[0] == 'Time:':
					if data:
						# publish
						try:
							msg = Imu()
							msg.header.stamp = rospy.Time.now()
							msg.header.frame_id = IMU_LINK
							msg.orientation = Quaternion(*data['Q'])
							for i in range(len(msg.orientation_covariance)):
								msg.orientation_covariance[i] = -1
							msg.angular_velocity = Vector3(*data['AV'])
							for i in range(len(msg.angular_velocity_covariance)):
								msg.angular_velocity_covariance[i] = -1
							msg.linear_acceleration = Vector3(*data['AV'])
							for i in range(len(msg.linear_acceleration_covariance)):
								msg.linear_acceleration_covariance[i] = -1
							self.pub_imu.publish(msg)
#							rospy.loginfo('%s' % data)
						except Exception, e:
							rospy.logwarn('%s' % e)
						data = {}
				elif line[0] == 'Q:':
					if not data:
						data['Q'] = str2float(line[1:])
				elif line[0] == 'AV:':
					if data:
						data['AV'] = str2float(line[1:])
				elif line[0] == 'LA:':
					if data:
						data['LA'] = str2float(line[1:])
		rospy.spin()

if __name__ == '__main__':

	node = TopSensors()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
