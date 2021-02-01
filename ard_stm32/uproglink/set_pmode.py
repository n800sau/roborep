#!/usr/bin/env python

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from serial import Serial

from dev import command_dev

s_port = command_dev
s_baud = 115200

ser = Serial(s_port, s_baud, timeout=5, writeTimeout=5)
#print s_port, s_baud

time.sleep(1)

for i in range(5):
	if ser.readline().strip() == 'Ready':
		ser.write('P')
		ser.flush()

		print ser.readline()
#		print ser.readline()
		break


