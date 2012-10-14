#!/usr/bin/env python

import serial
import struct
import time

ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=1)

def communicate(cmdline):
    global ser
    rs = None
#    print 'c' * len(cmdline), list(cmdline)
    v = struct.pack('c' * len(cmdline), *list(cmdline))
    for c in v:
        ser.write(c)
	time.sleep(0.01)
    ser.flush()
#    i += 2
    timeout = 0
    while True:
        line = ser.readline()
        if line:
#            for c in line:
#                print '%.2x ' % ord(c),
#            print
#            for c in line:
#                print '%.2c ' % c,
#            print
	    rs = line
        else:
    	    timeout += 1
#	    print 'timeout %d' % timeout
	    break
    return rs

def parse_reply(reply):
    if reply:
        cmd,parms = reply.split(':')
        parms = [5/1024. * int(p) for p in parms.split(',')]
    else:
	cmd,parms = None,[0,0,0,0]
    return cmd,parms

#ser = serial.Serial('/dev/ttyAMA0', 57600, timeout=1)
#ser = serial.Serial('/dev/ttyAMA0', 4800, timeout=None)
#i = 0
#timeout = 0
df = file('data.log', 'a')

while True:
    cmd1,parms1 = parse_reply(communicate('X<:.'))
    time.sleep(0.05)
    cmd2,parms2 = parse_reply(communicate('X>:.'))
    time.sleep(0.05)
    cmd3,parms3 = parse_reply(communicate('X{:.'))
    time.sleep(0.05)
    cmd4,parms4 = parse_reply(communicate('X}:.'))
    print cmd1,cmd2,cmd3,cmd4
    print parms1[0],parms2[0],parms3[0],parms4[0]
    print (parms1[0]-parms2[0])/0.5, (parms3[0]-parms4[0])/0.5
    print
    print >>df, '%s,%s,%s' % (time.strftime('%H:%M:%S', time.localtime(time.time())), (parms1[0]-parms2[0])/0.5, (parms3[0]-parms4[0])/0.5)
    df.flush()
    time.sleep(1)
#    print '%.2x ' % ord(ser.read(1))
#    a = array('B')
#    a.append(i%256)
#    ser.write(a.tostring())
#    v = struct.pack('cccc', 'X', '<', ':', '.')
#    for c in v:
#        ser.write(c)
#	time.sleep(0.01)
#    ser.flush()
#    i += 2
#    while True:
#        line = ser.readline()
#        if line:
#            for c in line:
#                print '%.2x ' % ord(c),
#            print
#            for c in line:
#                print '%.2c ' % c,
#            print
#	    print line
#        else:
#    	    timeout += 1
#	    print 'timeout %d' % timeout
#	    break
#    time.sleep(1)
    
#while True:
#    if ser.inWaiting():
#        for i in range(ser.inWaiting()):
#		c = ser.read(1)
#		print '%.2x ' % ord(c),
