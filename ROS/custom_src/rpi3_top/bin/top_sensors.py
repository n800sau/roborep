#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, random, time
import rospy
import serial
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3

IMU_LINK = 'imubox_link'
DEFPORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0'
# max timeout of no data before debug message is emitted
MAX_DATA_TIMEOUT = 10

def str2float(datalist):
	return [float(v) for v in datalist]

class TopSensors:

	def __init__(self):
		serport = rospy.get_param('sensors_serial_port', DEFPORT)
		self.ser = serial.Serial(serport, 115200)

	def run(self):
		rospy.init_node('top_sensors', anonymous = True)
		self.pub_imu = rospy.Publisher('imu', Imu, queue_size=10)
		self.pub_mag = rospy.Publisher('mag', MagneticField, queue_size=10)
		data = {}
		last_data_time = time.time()
		while not rospy.is_shutdown():
			line = self.ser.readline().strip()
			if line:
				last_data_time = time.time()
				line = line.split()
				try:
					if line[0] == 'Time:':
						if data:
							# publish
							try:
								msg = Imu()
								msg.header.stamp = rospy.Time.now()
								msg.header.frame_id = IMU_LINK
								msg.orientation = Quaternion(*data['Q'])
								for i in range(len(msg.orientation_covariance)):
									msg.orientation_covariance[i] = 0.1
								msg.angular_velocity = Vector3(*data['AV'])
								for i in range(len(msg.angular_velocity_covariance)):
									msg.angular_velocity_covariance[i] = 0.1
								msg.linear_acceleration = Vector3(*data['AV'])
								for i in range(len(msg.linear_acceleration_covariance)):
									msg.linear_acceleration_covariance[i] = 0.1
								self.pub_imu.publish(msg)
								msg = MagneticField()
								msg.header.stamp = rospy.Time.now()
								msg.header.frame_id = IMU_LINK
								msg.magnetic_field = Vector3(*[v / 1000. for v in data['M']])
								for i in range(len(msg.magnetic_field_covariance)):
									msg.magnetic_field_covariance[i] = 0.1
								self.pub_mag.publish(msg)
#								rospy.loginfo('top_sensors data published: %s' % data)
							except Exception, e:
								rospy.logwarn('Exception while publishing:%s' % e)
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
					elif line[0] == 'M:':
						if data:
							data['M'] = str2float(line[1:])
				except Exception, e:
					rospy.logwarn('Exception while parsing serial data:%s' % e)
					data = {}
			elif time.time() - last_data_time > MAX_DATA_TIMEOUT:
				rospy.loginfo('top_sensors NO data')
				last_data_time = time.time()
		rospy.spin()

if __name__ == '__main__':

	node = TopSensors()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
