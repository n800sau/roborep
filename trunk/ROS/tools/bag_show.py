#!/usr/bin/env python

import rosbag, sys, os, csv
import time

tlist = {
	'/sensors/l3g4200/axes':  '   gyro',
	'/sensors/adxl345/axes':  '        acc',
	'/sensors/hmc5883l/axes': '            compass',
	'/fchassis/state':        '                    sonar',
	'/fchassis/command':      '                          command',
}

#tlist = None

bagfname = sys.argv[1]

bag = rosbag.Bag(bagfname)

with open(bagfname + '.csv', 'w+') as csvfile:
	fcsv = csv.writer(csvfile, delimiter = ',')

	i = 0
	for topic, msg, t in bag.read_messages(topics=tlist.keys()):
		fcsv.writerow([t.secs, t.nsecs, tlist[topic]])
		i += 1
#		if i > 300:
#			break
	print '%d done' % i

bag.close()
