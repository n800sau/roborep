#!/usr/bin/env python

import serial
import struct
import time

class SerCom(serial.Serial):

    def eol_readline(self, eol='\n'):
	rs = ''
	r = None
	while r!= eol and r!='':
	    r = ser.read(1)
	    rs += r
	return rs

    def eol_readlines(self, eol='\n'):
	rs = []
	while True:
	    rep = ser.eol_readline(eol='\r')
	    if not rep:
		break
	    rs.append(rep)
	return rs


ser = SerCom('/dev/ttyAMA0', 19200, timeout=1)

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

#ser.write('AP 1 1\r')
#ser.flush()
#print ser.readline()

#ser.write('AT 1 1\r')
#ser.flush()
#print ser.readline()

#ser.write('SS 0 1 1500\r')
#ser.flush()
#print ser.readline()
