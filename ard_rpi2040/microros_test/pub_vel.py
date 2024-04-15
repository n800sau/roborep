#!/usr/bin/env python3

import sys, time

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.qos import qos_profile_system_default


def main(args=None):
	rclpy.init(args=args)

	node = rclpy.create_node('vel_publisher')
	publisher = node.create_publisher(TwistStamped, 'cmd_vel', qos_profile_system_default)

	r = node.create_rate(1)  # 1hz // the second spin_once makes that delay
	for l,a in ((0., 0.), (0.5, 1.), (-0.5, 1.), (0., 0.)):
		msg = TwistStamped()
		msg.header.stamp = node.get_clock().now().to_msg()
		msg.twist.linear.x = l
		msg.twist.angular.z = a
		publisher.publish(msg)
		rclpy.spin_once(node, timeout_sec=0.1)
		rclpy.spin_once(node)

	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
