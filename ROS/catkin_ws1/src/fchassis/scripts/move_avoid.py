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
		if data.sonar < 0.3:
			# back
			self.pub_command.publish(command='mboth', lpwm=50, lfwd=false, rpwm=50, rfwd=false)
			rospy.sleep(0.3)
			if random.choice([True, False]):
				self.pub_command.publish(command='mright', rpwm=50, rfwd=true)
			else:
				self.pub_command.publish(command='mleft', lpwm=50, lfwd=true)
		else:
			# forward
			self.pub_command.publish(command='mboth', lpwm=50, lfwd=true, rpwm=50, rfwd=true)


if __name__ == '__main__':


	try:
		MoveAvoid().run()
	except rospy.ROSInterruptException:
		pass
