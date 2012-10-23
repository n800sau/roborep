#!/usr/bin/env python

import serial

df = file('com.log', 'w')
ser = serial.Serial('/dev/ttyAMA0', 57600, timeout=3)
df.write(ser.readline())
ser.close()
df.close()

