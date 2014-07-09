#!/usr/bin/env python
import time

while True:
	print '1'
	file('/sys/class/gpio/gpio72/value', 'w').write('1')
	time.sleep(0.5)
	print '0'
	file('/sys/class/gpio/gpio72/value', 'w').write('0')
	time.sleep(0.5)

