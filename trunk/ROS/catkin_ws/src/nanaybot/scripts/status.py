#!/usr/bin/env python
import subprocess, re
import rospy
from nanaybot.msg import battery

TIMEOUT = 5.0

def status():
	pub = rospy.Publisher('/nanaybot/battery', battery)
	rospy.init_node('status_node')
	while not rospy.is_shutdown():
		output = subprocess.check_output(['acpi', '-b'])
		level = re.search(r'\s(\d+)%', output)
		if level:
			level = int(level.groups()[0])
		else:
			level = -1
		pub.publish(battery(level, output.find('Discharging') != -1, output))
		rospy.sleep(TIMEOUT)


if __name__ == '__main__':
	try:
		status()
	except rospy.ROSInterruptException:
		pass
