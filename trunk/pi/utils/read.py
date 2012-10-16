#!/usr/bin/env python

from sercom import SerCom
import struct
import time
from ser2multi import is_busy, m_on, m_off, picaxe_on

ser = SerCom('/dev/ttyAMA0', 19200, timeout=1)

def communicate(cmdline):
	rs = None
	if is_busy():
		time.sleep(1)
	else:
		global ser
		m_on()
		try:
			picaxe_on()
			v = struct.pack('c' * len(cmdline), *list(cmdline))
			for c in v:
				ser.write(c)
				time.sleep(0.005)
				ser.flush()
			ser.write('\r')
			rs = ser.eol_readline(eol='\r')
		finally:
			m_off()
	return rs

def parse_reply(reply):
#	print reply
	if reply:
		cmd,parms = reply.split(':')
		parms = [int(p) for p in parms.split(',')]
	else:
		cmd,parms = None,[0,0,0,0]
	return cmd,parms

df = file('data.log', 'a')

while True:
	cmd,parms = parse_reply(communicate('XV:\r'))
	if cmd == 'V':
		b1,b2,sw1,sw2 = [p/1000. for p in parms]
		print cmd
		print b1, b2, sw1, sw2
		bc = (b1 - b2) / 0.05
		swc = (sw1 - sw2) / 0.05
		print bc, swc
		print
		print >>df, '%s,%s,%s,%s,%s,%s,%s' % (time.strftime('%Y.%m.%d %H:%M:%S', time.localtime(time.time())), bc, b1, b2, swc, sw1, sw2)
		df.flush()
	time.sleep(1)
