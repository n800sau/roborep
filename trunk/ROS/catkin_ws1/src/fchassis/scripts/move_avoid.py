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
		self.pub_command = rospy.Publisher('/fchassis/command', msg.command, queue_size=1)
#		rospy.loginfo('Move back')
#		self.pub_command.publish(command='mboth', lpwr=50, lfwd=False, rpwr=50, rfwd=False, timeout=0.5)
		rospy.sleep(10)
		rospy.loginfo('Here 1')
		self.pub_command.publish(command='walk_around', pwr=90, timeout=30)
		rospy.loginfo('Here 2')
#		self.commandlist = [
#			dict(command='walk_around', pwr=90, timeout=30)
#		]
#		self.subs = rospy.Subscriber("/fchassis/state", msg.state, self.wait_move_state)
		rospy.spin()

	def wait_move_state(self, state):
		if state.lpwr > 0 and state.rpwr > 0:
			rospy.loginfo('Moving')
			self.subs.unregister()
			self.subs = rospy.Subscriber("/fchassis/state", msg.state, self.on_state)

	def on_state(self, state):
		if state.lpwr == 0 and state.rpwr == 0:
			rospy.loginfo('Stopped')
			if len(self.commandlist):
				cmd = self.commandlist.pop(-1)
				self.pub_command.publish(**cmd)
				rospy.loginfo('Send cmd:' + cmd)
			else:
				self.subs.unregister()

	def on_state1(self, data):
		if data.sonar > 0:
			if data.sonar < 0.5:
				# back
				self.direction = 'back'
				self.pub_command.publish(command='mboth', lpwr=40, lfwd=False, rpwr=40, rfwd=False)
				rospy.loginfo('Back')
				rospy.sleep(2)
				if random.choice([True, False]):
					self.direction = 'right'
					self.pub_command.publish(command='mright', rpwr=50, rfwd=False)
					rospy.loginfo('Right')
					rospy.sleep(2)
				else:
					self.direction = 'left'
					self.pub_command.publish(command='mleft', lpwr=50, lfwd=False)
					rospy.loginfo('Left')
					rospy.sleep(2)
			else:
				# forward
				self.direction = 'forward'
				self.pub_command.publish(command='mboth', lpwr=50, lfwd=True, rpwr=50, rfwd=True)
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
