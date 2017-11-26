#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, random, time
import rospy
import serial
from math import sin, cos, pi
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

DEFPORT = '/dev/ttyS1'
# max timeout of no data before debug message is emitted
MAX_DATA_TIMEOUT = 10

def str2float(datalist):
	return [float(v) for v in datalist]

class BlueWheel:

	def __init__(self):
		serport = rospy.get_param('bwh_serial_port', DEFPORT)
		self.ser = serial.Serial(serport, 115200)

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
		self.ser.write("set m1vel %d\n" % (left,))
		self.ser.write("set m2vel %d\n" % (right,))
		self.ser.flush()
		reply = self.ser.read()
		rospy.loginfo('reply: %s' % reply)

	def run(self):
		rospy.init_node('blue_wheel', anonymous = True)

		# Subscriptions
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

		# Publications
		pub_odom = rospy.Publisher('', Imu, queue_size=10)
		pub_odom = rospy.Publisher("odom", Odometry, queue_size=50)
		odom_broadcaster = tf.TransformBroadcaster()

		last_time = rospy.Time.now()
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			current_time = rospy.Time.now()

			# compute odometry in a typical way given the velocities of the robot
			dt = (current_time - last_time).to_sec()
			delta_x = (vx * cos(th) - vy * sin(th)) * dt
			delta_y = (vx * sin(th) + vy * cos(th)) * dt
			delta_th = vth * dt

			x += delta_x
			y += delta_y
			th += delta_th

			# since all odometry is 6DOF we'll need a quaternion created from yaw
			odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

			# first, we'll publish the transform over tf
			odom_broadcaster.sendTransform(
						(x, y, 0.),
						odom_quat,
						current_time,
						"base_link",
						"odom"
			)

			# next, we'll publish the odometry message over ROS
			odom = Odometry()
			odom.header.stamp = current_time
			odom.header.frame_id = "odom"

			# set the position
			odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

			# set the velocity
			odom.child_frame_id = "base_link"
			odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

			# publish the message
			pub_odom.publish(odom)

			last_time = current_time

			r.sleep()

if __name__ == '__main__':

	node = BlueWheel()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
