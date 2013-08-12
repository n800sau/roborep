#!/usr/bin/env python
import serial

ser = serial.Serial(
	port='/dev/ttyUSB0',
    baudrate=4800
)

ser.open()
show = False
for line in ser:
#	print line
	l = line.split(':')
	if len(l) == 2:
		if l[0] == 'V':
			show = True
		if show:
			r,v = [v.strip() for v in l]
			if r == 'V':
				print 'V:%g' % (float(v) / 100),
			else:
				t = {'0': 'X', '2': 'Y', '4': 'Z'}.get(r, None)
				if t:
					v = int(v)
					if v > 0x8000:
						v -= 0x10000
					print '%s = %d' % (t, v),
				if t != 'Z':
					print ',',
				else:
					print
#	elif not l:
#		print


