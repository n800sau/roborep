#!/usr/bin/env python
import rospy
from oculus2wd.msg import drive

def talker():
    pub = rospy.Publisher('/oculus_base_command', drive)
    rospy.init_node('drive_node')
    while not rospy.is_shutdown():
        pub.publish(drive(ord('b'), 10, 1))
        print 'published'
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
