#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, random
import rospy
from fchassis import msg

class MoveAvoid:

	def run(self):
		rospy.init_node('move_avoid', anonymous = True)
		rospy.Subscriber("/fchassis/state", msg.state, self.on_state)
		self.pub_command = rospy.Publisher('/fchassis/command', msg.command, queue_size=1)
		rospy.spin()

	def on_state(self, data):
		if data.sonar > 0:
			if data.sonar < 0.5:
				# back
				self.pub_command.publish(command='mboth', lpwm=40, lfwd=True, rpwm=40, rfwd=True)
				rospy.sleep(0.2)
				if random.choice([True, False]):
					self.pub_command.publish(command='mright', rpwm=50, rfwd=True)
					rospy.sleep(0.2)
				else:
					self.pub_command.publish(command='mleft', lpwm=50, lfwd=True)
					rospy.sleep(0.2)
			else:
				# forward
				self.pub_command.publish(command='mboth', lpwm=50, lfwd=False, rpwm=50, rfwd=False)
				rospy.sleep(0.2)
		else:
			self.pub_command.publish(command='mstop')


if __name__ == '__main__':


	node = MoveAvoid()
	try:
		node.run()
	except rospy.ROSInterruptException:
		raise
