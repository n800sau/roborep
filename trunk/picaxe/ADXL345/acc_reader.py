#!/usr/bin/env python
import serial

ser = serial.Serial(
	port='/dev/ttyUSB0',
    baudrate=4800
)

ser.open()
for line in ser:
	print line
	l = line.split(':')
	if len(l) == 2:
		r,v = [v.strip() for v in l]
		t = {'0': 'X', '2': 'Y', '4': 'Z'}.get(r, None)
		if t:
			v = int(v)
			if v > 0x8000:
				v -= 0x10000
			print t, '=%4.4X' % v,
		if t != 'Z':
			print ',',
		else:
			print
#	elif not l:
#		print


