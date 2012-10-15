#!/usr/bin/env python

from sercom import SerCom
import struct
import time

ser = SerCom('/dev/ttyAMA0', 19200, timeout=1)

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
#    timeout = 0
#    while True:
    rs = ser.eol_readline(eol='\r')
#        if line:
#            for c in line:
#                print '%.2x ' % ord(c),
#            print
#            for c in line:
#                print '%.2c ' % c,
#            print
#	    rs = line
#        else:
#    	    timeout += 1
#	    print 'timeout %d' % timeout
#	    break
    return rs

def parse_reply(reply):
    if reply:
        cmd,parms = reply.split(':')
        parms = [int(p) for p in parms.split(',')]
    else:
	cmd,parms = None,[0,0,0,0]
    return cmd,parms

#ser = serial.Serial('/dev/ttyAMA0', 57600, timeout=1)
#ser = serial.Serial('/dev/ttyAMA0', 4800, timeout=None)
#i = 0
#timeout = 0
df = file('data.log', 'a')

#< - BATT1
#> - BATT2
#{ - SW1
#} - SW2

while True:
    cmd,parms = parse_reply(communicate('XV:\r'))
#    time.sleep(0.05)
#    cmd2,parms2 = parse_reply(communicate('X>:\r'))
#    time.sleep(0.05)
#    cmd3,parms3 = parse_reply(communicate('X{:\r'))
#    time.sleep(0.05)
#    cmd4,parms4 = parse_reply(communicate('X}:\r'))
    print cmd
    print parms[0]/1000.,parms[1]/1000.,parms[2]/1000.,parms[3]/1000.
    print (parms[0]-parms[1])/1000./0.05, (parms[2]-parms[3])/1000./0.05
#    print
#    print >>df, '%s,%s,%s' % (time.strftime('%m:%d %H:%M:%S', time.localtime(time.time())), (parms1[0]-parms2[0])/1000./0.05, (parms3[0]-parms4[0])/1000./0.05)
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
