#!/usr/bin/env python

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
from serial import Serial

cfgobj = configure(os.path.join(os.path.dirname(__file__)))

cfg = cfgobj.as_dict('serial')

s_port = cfg['serial_port']
s_baud = int(cfg.get('baud_rate', 115200))

ser = Serial(s_port, s_baud, timeout=5, writeTimeout=5)
#print s_port, s_baud

for i in range(5):
	if ser.readline().strip() == 'Ready':

		val = raw_input('Input:')
		ser.write(val)
		ser.flush()

		print ser.readline()
		print ser.readline()
		break
	else:
		time.sleep(1)


