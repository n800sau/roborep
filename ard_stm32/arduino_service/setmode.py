#!/usr/bin/env python

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from serial import Serial

vars_sh = open('vars.sh').read()
exec(vars_sh)

ser = Serial(DEV, 115200, timeout=5, writeTimeout=5)
#print s_port, s_baud

for i in range(5):
	if ser.readline().strip() == 'Ready':

		val = raw_input('Input:')
		if not val:
			val = "#"
		ser.write(val)
		ser.flush()

		print ser.readline()
		print ser.readline()
		break
	else:
		time.sleep(1)


