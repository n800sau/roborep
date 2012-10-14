#!/usr/bin/env python

import serial
import struct
import time

ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=1)

ser.write('GV\r')
ser.flush()
print ser.read()

ser.write('GW\r')
ser.flush()
print ser.read()

#ser.write('AP 1 1\r')
#ser.flush()
#print ser.readline()

#ser.write('AT 1 1\r')
#ser.flush()
#print ser.readline()

ser.write('SS 0 1 1500\r')
ser.flush()
print ser.readline()
