#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, signal, copy
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import pandas as pd

def last_inverse_value(data_secs, data):
#	print data_secs[:10].dtype
	ts = pd.Series(data, index=data_secs)
#	print ts[:2]
	# 100 ns resample
	ts = ts.resample('100L', how='mean')
	data = ts.values
	data_secs = np.float64(ts.keys().values) / 1000000000
#	print '@@@@@@', data_secs[0], data[0]
#	data = np.interp(np.arange(data_secs[0], data_secs[-1]), data_secs, data)
#	print '@@@@@@', data.shape
#	df = np.diff(data_secs)
#	print '!!!', df
#	sampling_length = np.mean(df) # measured every 0.1 sec 
#	print '###', sampling_length
	freq = np.fft.fftfreq(len(data), d = 0.1)
	fft = np.fft.fft(data)
	x = freq[:len(data)/2]
	for i in range(len(x)):
		if x[i] > 0.005: # cut off all frequencies higher than 0.005
			fft[i] = 0.0
			fft[len(data)/2 + i] = 0.0
	inverse = np.fft.ifft(fft)
#	print '#######',data[0], data_secs[0]
#	rs = np.empty((data_secs.shape[0], 2), dtype=np.float64)
#	rs[:, 0] = data_secs % 3600
#	rs[:, 1] = inverse
	return inverse[-1]

WINDOW_SIZE = 100
mcount = 0
data = np.zeros((WINDOW_SIZE, 1 + 3 + 3 + 4), dtype=np.float64)

def callback(msg):
	global data, mcount
	# collect window
	data = np.roll(data, -1, axis=0)
#	print 'type=', dir(msg.header.stamp), msg.header.stamp
#	print msg.header.stamp.secs * 1000000000 + msg.header.stamp.nsecs
	data[-1, 0] = msg.header.stamp.secs * 1000000000 + msg.header.stamp.nsecs
	data[-1, 1] = msg.linear_acceleration.x
	data[-1, 2] = msg.linear_acceleration.y
	data[-1, 3] = msg.linear_acceleration.z
	data[-1, 4] = msg.angular_velocity.x
	data[-1, 5] = msg.angular_velocity.y
	data[-1, 6] = msg.angular_velocity.z
	data[-1, 7] = msg.orientation.x
	data[-1, 8] = msg.orientation.y
	data[-1, 9] = msg.orientation.z
	data[-1, 10] = msg.orientation.w
	mcount += 1
	if mcount >= WINDOW_SIZE:
		data_secs = np.array(data[:, 0], dtype='datetime64[ns]')
#		print data_secs
		data_out = []
		for i in range(1, data.shape[1]):
			data_out.append(last_inverse_value(data_secs, data[:,i]))
		msg.linear_acceleration.x = data_out[0]
		msg.linear_acceleration.y = data_out[1]
		msg.linear_acceleration.z = data_out[2]
		msg.angular_velocity.x = data_out[3]
		msg.angular_velocity.y = data_out[4]
		msg.angular_velocity.z = data_out[5]
		msg.orientation.x = data_out[6]
		msg.orientation.y = data_out[7]
		msg.orientation.z = data_out[8]
		msg.orientation.w = data_out[9]
		pub.publish(msg)
#		rospy.signal_shutdown("bye")

rospy.init_node('imu_fft', anonymous = True)

rospy.Subscriber("/oculus2wd/imu_raw", Imu, callback)
pub = rospy.Publisher('/oculus2wd/imu_005', Imu)




#data = np.loadtxt('imu.csv', delimiter = ',', skiprows = 1, usecols = (0,29), dtype=np.float64)

# nanosecs // 1000 000 000 = secs
# data_secs = np.float64(np.int64(data[:, 0]) // 1000000 % 1000000) / 1000
#print data[:, 0][:10]
#data_secs = np.array(data[:, 0], dtype='datetime64[ns]')
#print data_secs[:10]
#data = data[:,1]

#data_out = make_inverse(data_secs, data)

#np.savetxt('imu_plot.txt', data_out, '%f')

rospy.spin()
