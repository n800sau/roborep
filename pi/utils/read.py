#!/usr/bin/env python

from sercom import SerCom
import struct
import time

ser = SerCom('/dev/ttyAMA0', 19200, timeout=1)

def communicate(cmdline):
    global ser
    v = struct.pack('c' * len(cmdline), *list(cmdline))
    for c in v:
        ser.write(c)
	time.sleep(0.005)
    ser.flush()
    return ser.eol_readline(eol='\r')

def parse_reply(reply):
    if reply:
        cmd,parms = reply.split(':')
        parms = [int(p) for p in parms.split(',')]
    else:
	cmd,parms = None,[0,0,0,0]
    return cmd,parms

df = file('data.log', 'a')

while True:
    cmd,parms = parse_reply(communicate('XV:\r'))
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
