#!/usr/bin/env python3

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from serial import Serial

exec(open("vars.sh").read())

s_port = DEV
s_baud = 115200

ser = Serial(s_port, s_baud, timeout=5, writeTimeout=5)
#print(s_port, s_baud)

for i in range(5):
	if ser.readline().strip() == b'Ready':

		val = input('Input:')
		if not val:
			val = b"#"
		ser.write(val)
		ser.flush()

		print(ser.readline())
		print(ser.readline())
		break
	else:
		time.sleep(1)


