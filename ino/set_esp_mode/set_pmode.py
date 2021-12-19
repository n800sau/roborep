#!/usr/bin/env python3

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from serial import Serial

exec(open(os.path.join(os.path.dirname(__file__), "vars.sh")).read())

s_port = DEV
s_baud = 115200

ser = Serial(s_port, s_baud, timeout=5, writeTimeout=5)
ser.write(b'?')
ser.flush()

for i in range(5):
	if ser.readline().decode().strip() == 'Ready':
		ser.write(b'p')
		ser.flush()

		for i in range(100):
			line = ser.readline().decode().strip()
			print(line)
			if line == 'end':
				break
		break
