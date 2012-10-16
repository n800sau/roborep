#!/usr/bin/env python

import serial
import struct
import time
from sercom import SerCom
from ser2multi import is_busy, m_on, m_off, camera_on

ser = SerCom('/dev/ttyAMA0', 19200, timeout=1)

while True:
	if is_busy():
		time.sleep(0.1)
		continue
	m_on()
	camera_on()
	try:
		i = 1
		for cmd in (
				'GV',
				'GW',
				'GI',
				'GS 0',
				'GS 1',
				'TM 1',
				):
			ser.flushInput()
			ser.write(cmd + '\r')
			ser.flushOutput()
			print cmd
			i += 1
			print '\n'.join(ser.eol_readlines(eol='\r'))
			print
		break
	finally:
		m_off()

#ser.write('AP 1 1\r')
#ser.flush()
#print ser.readline()

#ser.write('AT 1 1\r')
#ser.flush()
#print ser.readline()

#ser.write('SS 0 1 1500\r')
#ser.flush()
#print ser.readline()
