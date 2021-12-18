#!/usr/bin/env python

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from serial import Serial

execfile("vars.sh")

s_port = DEV
s_baud = 115200

ser = Serial(s_port, s_baud, timeout=5, writeTimeout=5)
#print s_port, s_baud

for i in range(5):
	if ser.readline().strip() == 'Ready':

		while True:
			val = raw_input('Input:')
			if val.upper() == 'X':
				print('Exit')
				break
			if not val:
				val = "#"
			ser.write(val)
			ser.flush()

			for i in range(100):
				line = ser.readline().strip()
				print(line)
				if line == 'end':
					break
		break
	else:
		time.sleep(1)
