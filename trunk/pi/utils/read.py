#!/usr/bin/env python

import serial
import struct
import time

ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=1)
#ser = serial.Serial('/dev/ttyAMA0', 57600, timeout=1)
#ser = serial.Serial('/dev/ttyAMA0', 4800, timeout=None)
#i = 0
timeout = 0
while True:
#    print '%.2x ' % ord(ser.read(1))
#    a = array('B')
#    a.append(i%256)
#    ser.write(a.tostring())
    v = struct.pack('ccc', 'X', '<', '.')
    for c in v:
        ser.write(c)
	time.sleep(0.01)
    ser.flush()
#    i += 2
    while True:
        line = ser.readline()
        if line:
            for c in line:
                print '%.2x ' % ord(c),
            print
            for c in line:
                print '%.2c ' % c,
            print
        else:
    	    timeout += 1
	    print 'timeout %d' % timeout
	    break
    time.sleep(1)
    
#while True:
#    if ser.inWaiting():
#        for i in range(ser.inWaiting()):
#		c = ser.read(1)
#		print '%.2x ' % ord(c),
