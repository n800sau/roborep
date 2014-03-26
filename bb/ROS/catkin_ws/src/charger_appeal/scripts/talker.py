#!/usr/bin/env python
import rospy
from charger_appeal.msg import lighthouse_cmd

def talker():
    pub = rospy.Publisher('/charger_appeal/lighthouse_cmd', lighthouse_cmd)
    rospy.init_node('charger_light_commander')
    while not rospy.is_shutdown():
        pub.publish(lighthouse_cmd("on", 10))
        print 'published'
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
