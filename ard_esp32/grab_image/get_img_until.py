#!/usr/bin/env python

import time, os
from serial import Serial

execfile(os.path.join(os.path.dirname(__file__), "vars.sh"))

s_port = DEV
s_baud = 115200

ser = Serial(s_port, s_baud, timeout=5, writeTimeout=5)
#print s_port, s_baud

time.sleep(1)

DIVIDER = '#----------#\r'

i = 0
while True:
	bytes = ser.read_until(DIVIDER)
	if bytes.endswith(DIVIDER):
		jpg = ser.read_until(DIVIDER)
		if jpg.endswith(DIVIDER):
			ofname = 'out_%04d.jpg' % i
			open(ofname, 'wb').write(jpg[:-len(DIVIDER)])
			i += 1
			print(ofname + ' written')
			#break
		else:
			time.sleep(0.01)
	else:
		time.sleep(0.1)
