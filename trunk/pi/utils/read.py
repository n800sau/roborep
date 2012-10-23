#!/usr/bin/env python

from sercom import SerCom
import struct
import time
from ser2multi import is_busy, m_on, m_off, picaxe_on

ser = SerCom('/dev/ttyAMA0', 19200, timeout=1)

b1list = []
b2list = []
sw1list = []
sw2list = []

def communicate(cmdline):
	rs = None
	if is_busy():
		time.sleep(1)
	else:
		global ser
		m_on()
		time.sleep(0.005)
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

pb1=0
pb2=0
psw1=0
psw2=0
mpb1=-1
mpb2=-1
mpsw1=-1
mpsw2=-1

doit=False
while True:
	cmd,parms = parse_reply(communicate('XV:\r'))
	if cmd == 'V':
		b1,b2,sw1,sw2 = parms
		print >>df, '%s,%s,%s,%s,%s' % (time.strftime('%Y.%m.%d %H:%M:%S', time.localtime(time.time())), b1, b2, sw1, sw2)
		df.flush()
		print b1, b2, sw1, sw2
		b1list.append(b1)
		b2list.append(b2)
		sw1list.append(sw1)
		sw2list.append(sw2)
		del b1list[:-10]
		del b2list[:-10]
		del sw1list[:-10]
		del sw2list[:-10]
		if len(b1list) >= 10:
			b1=float(sum(b1list))/len(b1list)
			b2=float(sum(b2list))/len(b2list)
			sw1=float(sum(sw1list))/len(sw1list)
			sw2=float(sum(sw2list))/len(sw2list)
		#2.85,2.88,2.67,2.67
#		b1,b2,sw1,sw2 = [p/1000.*4096 for p in parms]
			print cmd
			print b1, b2, sw1, sw2
			print mpb1,mpb2,mpsw1,mpsw2
#			bc = (b1 - b2) / 0.05
#			swc = (sw1 - sw2) / 0.05
#			print bc, swc
			print
			if doit:
				mpb1=max(abs(b1-pb1), mpb1) 
				mpb2=max(abs(b2-pb2), mpb2)
				mpsw1=max(abs(sw1-psw1), mpsw1)
				mpsw2=max(abs(sw2-psw2), mpsw2)
			else:
				doit = True
			pb1=b1
			pb2=b2
			psw1=sw1
			psw2=sw2
	time.sleep(1)
