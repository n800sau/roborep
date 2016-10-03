#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, random
import rospy
from fchassis import msg

class MoveAvoid:

	def __init__(self):
		self.direction = None
		self.change_time = None

	def run(self):
		rospy.init_node('move_avoid', anonymous = True)
#		rospy.Subscriber("/fchassis/state", msg.state, self.on_state)
		self.pub_command = rospy.Publisher('/fchassis/command', msg.command, queue_size=1)
		rospy.sleep(10)
		self.pub_command.publish(command='mboth', lpwm=40, lfwd=False, rpwm=40, rfwd=False)
		rospy.sleep(10)
		self.pub_command.publish(command='walk_around', lpwm=50, rpwm=50)
		rospy.loginfo('Start walking')
		rospy.sleep(60)
		self.pub_command.publish(command='mstop')
		rospy.loginfo('Stop')
#		rospy.signal_shutdown("Walk around shutdown")

	def on_state(self, data):
		if data.sonar > 0:
			if data.sonar < 0.5:
				# back
				self.direction = 'back'
				self.pub_command.publish(command='mboth', lpwm=40, lfwd=False, rpwm=40, rfwd=False)
				rospy.loginfo('Back')
				rospy.sleep(2)
				if random.choice([True, False]):
					self.direction = 'right'
					self.pub_command.publish(command='mright', rpwm=50, rfwd=False)
					rospy.loginfo('Right')
					rospy.sleep(2)
				else:
					self.direction = 'left'
					self.pub_command.publish(command='mleft', lpwm=50, lfwd=False)
					rospy.loginfo('Left')
					rospy.sleep(2)
			else:
				# forward
				self.direction = 'forward'
				self.pub_command.publish(command='mboth', lpwm=50, lfwd=True, rpwm=50, rfwd=True)
				rospy.loginfo('Forward')
				rospy.sleep(2)
		else:
			rospy.loginfo('Stop')
			self.direction = 'stop'
			self.pub_command.publish(command='mstop')


if __name__ == '__main__':


	node = MoveAvoid()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
