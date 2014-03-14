#!/usr/bin/env python
import rospy
from cmvision.msg import Blobs


def callback(msg, pub):
	print 'Count=%d' % msg.blob_count
	if msg.blob_count > 0:
		pub.publish(msg)


if __name__ == '__main__':
	rospy.init_node('msg_filter', anonymous = True)
	pub = rospy.Publisher('/blobs_filtered', Blobs)
	rospy.Subscriber("/blobs", Blobs, callback, pub)
	rospy.spin()
