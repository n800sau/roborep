#!/usr/bin/env python

import serial
import struct
import time
from sercom import SerCom
from ser2multi import is_busy, m_on, m_off, camera_on

ser = SerCom('/dev/ttyAMA0', 19200, timeout=2, interCharTimeout=None)

def request(cmdline):
	ser.flushInput()
	ser.write(cmdline + '\r')
	ser.flushOutput()
	return ser.eol_readlines(eol='\r', mincount=1)

while True:
	if is_busy():
		time.sleep(0.1)
		continue
	m_on()
	camera_on()
	try:
		i = 1
		for cmd in (
#				'RS',
#				'SB',
#				'AT 0 0',
#				'GW',
#				'ST',
#				'ST 100 200 100 200 100 200',
#				'ST 52 64 218 240 0 22 11 12',
#				'GT',
#				'SW 0 0 79 30',
#				'GM',
#				'SS 1 1 1000', #center vert
#				'SS 0 1 1400', #center horiz
				'SS 1 1 1000',
				'SS 0 1 1400',
#				'GI',
#				'GS 0',
#				'GS 1',
#				'TM 1',
#				'DI',
				):
			print cmd
			reply = request(cmd)
			i += 1
			print '\n'.join(reply)
			print
#			time.sleep(1)
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
