#!/usr/bin/env python

import rosbag, sys, os, csv
import time

bagfname = sys.argv[1]
#bagfname = 'record_2016-10-14T09:29+1100.bag' if len(sys.argv) < 2 else sys.argv[1]

tlist = {
	'/fchassis/state':   '             state',
	'/fchassis/mf':      '          mf',
	'/fchassis/imu':     '     imu_raw',
	'/fchassis/sonar':   'sonar',
	'/imu/data':         '                 imu',
}

def proc_line(msg):
	return [
#		'%.2f' % msg.sonar,
		msg.command,
		'%d' % msg.single_tap,
		'%d' % msg.lpwr,
		'%d' % msg.rpwr,
		'%d' % msg.pwroffset,
#		'%d' % msg.lcurrent,
#		'%d' % msg.rcurrent,
		'%d' % msg.lcount,
		'%d' % msg.rcount,
		'%d' % msg.heading,
#		'%.4f' % msg.compass.x,
#		'%.4f' % msg.compass.y,
#		'%.4f' % msg.compass.z,
#		'%d' % msg.gyro.x,
#		'%d' % msg.gyro.y,
#		'%d' % msg.gyro.z,
#		'%.4f' % msg.acc.x,
#		'%.4f' % msg.acc.y,
#		'%.4f' % msg.acc.z,
	]

#tlist = None


bag = rosbag.Bag(bagfname)

with open(bagfname + '_x.csv', 'w+') as csvfile:
	fcsv = csv.writer(csvfile, delimiter = ',')

	i = 0
	active = True
	for topic, msg, t in bag.read_messages(topics=tlist.keys()):
		name = tlist[topic]
		vals = []
		if name.strip() == 'state':
			vals = proc_line(msg)
		if name.strip() == 'sonar':
			vals = ['%.2f' % msg.range]
		if name.strip() in ('imu', 'imu_raw'):
			vals = [
				'%.2f' % msg.linear_acceleration.x,
				'%.2f' % msg.linear_acceleration.y,
				'%.2f' % msg.linear_acceleration.z,
				'%.2f' % msg.angular_velocity.x,
				'%.2f' % msg.angular_velocity.y,
				'%.2f' % msg.angular_velocity.z,
			]
		if name.strip() == 'mf':
			vals = [
				'%.2f' % msg.magnetic_field.x,
				'%.2f' % msg.magnetic_field.y,
				'%.2f' % msg.magnetic_field.z,
			]
		elif name.strip() == 'command':
			active = True
			vals = [msg.command]
		if vals and active:
			fcsv.writerow([t.secs, t.nsecs] + vals + [name])
		i += 1
#		if i > 300:
#			break
	print '%d done' % i

bag.close()
