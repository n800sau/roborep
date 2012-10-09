#!/usr/bin/env python

import serial
from array import array

ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=1)
#ser = serial.Serial('/dev/ttyAMA0', 57600, timeout=1)
#ser = serial.Serial('/dev/ttyAMA0', 4800, timeout=None)
i = 1
while True:
#    print '%.2x ' % ord(ser.read(1))
    a = array('B')
    a.append(i%256)
    ser.write(a.tostring())
#    ser.write('X<\x0d')
    ser.flush()
    i += 2
    print '%s' % ser.readline(),
    
#while True:
#    if ser.inWaiting():
#        for i in range(ser.inWaiting()):
#		c = ser.read(1)
#		print '%.2x ' % ord(c),
