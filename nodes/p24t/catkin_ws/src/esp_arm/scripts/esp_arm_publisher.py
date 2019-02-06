#!/usr/bin/python
import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

GRIP_BUTTON = 6

UPPER_BUTTON = 5

YAW_AXES = 0
SHOULDER_AXES = 1

class EspArmPublisher:


	def __init__(self):
		self.position = {
			'grip': 0,
			'upper': 0,
			'lower': 0,
			'yaw': 0
		}
		self.arm_sub = rospy.Subscriber("/joy_arm", Joy, self.arm_joy_callback)
		self.pub_grip = rospy.Publisher("/grip", Int16, queue_size=1)
		self.pub_yaw = rospy.Publisher("/yaw", Int16, queue_size=1)
		self.pub_upper = rospy.Publisher("/lower", Int16, queue_size=1)
		self.pub_lower = rospy.Publisher("/upper", Int16, queue_size=1)

	def publish_messages(self, data):
	  # Execute arm position
		rospy.loginfo('position:%s\n' % self.position)
		self.pub_grip.publish(self.position['grip'])
		self.pub_yaw.publish(-1 if int(data.axes[YAW_AXES]) == 0 else self.position['yaw'])
		self.pub_upper.publish(-1 if int(data.axes[SHOULDER_AXES]) == 0 else self.position['upper'])
		self.pub_lower.publish(-1 if int(data.axes[SHOULDER_AXES]) == 0 else self.position['lower'])

	def arm_joy_callback(self, data):

		self.position['grip'] = 100 if data.buttons[GRIP_BUTTON] else 0

		self.position['yaw'] += data.axes[YAW_AXES]
		if data.buttons[UPPER_BUTTON]:
			self.position['upper'] += data.axes[SHOULDER_AXES]
		else:
			self.position['lower'] += data.axes[SHOULDER_AXES]

		# MOVE ARM
		self.publish_messages(data)


def main():
	rospy.init_node("esp_arm_publisher")
	controller = EspArmPublisher()
	rospy.spin()

main()
