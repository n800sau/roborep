#!/usr/bin/python
import rospy
import copy

from sensor_msgs.msg import Joy
from esp_arm.srv import ArmServo

GRIP_BUTTON = 6

UPPER_BUTTON = 5

YAW_AXES = 0
SHOULDER_AXES = 1

class EspArmServiceClient:


	def __init__(self):
		self.new_position = {
			'grip': 100,
			'upper': 50,
			'lower': 50,
			'yaw': 50
		}
		self.position = dict([(k, 0) for k in self.new_position])
		self.velocity = dict([(k, 0) for k in self.new_position])
		self.arm_sub = rospy.Subscriber("/joy_arm", Joy, self.arm_joy_callback)

	def update_position(self):
		for k in ('upper', 'lower', 'yaw'):
			self.new_position[k] = min(100, max(0, self.new_position[k]+self.velocity[k]))
		self.call_services()

	def call_services(self):
		# Execute arm position
#		rospy.loginfo('new position:%s\n' % self.new_position)
		for k,v in self.new_position.items():
			if self.position[k] != v:
				rospy.loginfo('new %s position:%s\n' % (k, self.new_position[k]))
				rospy.wait_for_service(k)
				try:
					service = rospy.ServiceProxy(k, ArmServo)
					if self.velocity[k] == 0:
						response = service(wvalue=-1)
					else:
						response = service(wvalue=v, msecs=500)
					rospy.loginfo('%s response: %s' % (k, response))
				except rospy.ServiceException, e:
					rospy.logerror("Service call failed: %s" % e)
		self.position = copy.copy(self.new_position)

	def arm_joy_callback(self, data):

		self.new_position['grip'] = 100 if data.buttons[GRIP_BUTTON] else 1

		self.velocity['yaw'] = data.axes[YAW_AXES]
		if data.buttons[UPPER_BUTTON]:
			self.velocity['upper'] = data.axes[SHOULDER_AXES]
		else:
			self.velocity['lower'] = data.axes[SHOULDER_AXES]

def main():
	rospy.init_node("esp_arm_client")
	controller = EspArmServiceClient()
	r = rospy.Rate(10) # 10hz 
	while not rospy.is_shutdown():
		controller.update_position()
		r.sleep()

main()
