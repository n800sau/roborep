#!/usr/bin/env python

import rosbag, sys, os, csv
import time

tlist = {
	'/sensors/l3g4200/axes':  '   gyro',
	'/sensors/adxl345/axes':  '        acc',
	'/sensors/hmc5883l/heading': '            compass',
	'/fchassis/state':        '                    state',
	'/fchassis/command':      '                          command',
}

#tlist = None

bagfname = sys.argv[1]

bag = rosbag.Bag(bagfname)

with open(bagfname + '.csv', 'w+') as csvfile:
	fcsv = csv.writer(csvfile, delimiter = ',')

	i = 0
	for topic, msg, t in bag.read_messages(topics=tlist.keys()):
		name = tlist[topic]
		vals = []
		if name.strip() == 'state':
			vals = ['%.2f' % msg.sonar]
		elif name.strip() == 'compass':
			vals = ['%d' % msg.heading]
		elif name.strip() == 'command':
			vals = [msg.command]
		elif name.strip() in ('acc', 'gyro'):
			vals = ['%d' % msg.vector.x , '%d' % msg.vector.y, '%d' % msg.vector.z]
		fcsv.writerow([t.secs, t.nsecs, name] + vals)
		i += 1
#		if i > 300:
#			break
	print '%d done' % i

bag.close()
