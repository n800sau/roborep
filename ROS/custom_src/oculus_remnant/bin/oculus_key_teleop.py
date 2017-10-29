#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = 0.0
turn = 0.0

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	rospy.init_node('oculus_key_teleop')

	try:
		while True:
			key = getKey()
			if key == 'w':
				speed += 0.1
			elif key == 'a':
				turn += 0.2
			elif key == 'd':
				turn += -0.2
			elif key == 'x':
				speed += -0.1
			elif key in (' ', 's'):
				turn = speed = 0
			print "%s\rs: %.1f\tt: %.1f\r" % (' ' * 50, speed, turn),
			if key == '\x1b':
					break

			twist = Twist()
			twist.linear.x = speed
			twist.linear.y = 0
			twist.linear.z = 0
			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = turn
			pub.publish(twist)

	except:
		print e

	finally:
		print "%s\rs: %.1f\tt: %.1f\r" % (' ' * 50, 0, 0),
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


