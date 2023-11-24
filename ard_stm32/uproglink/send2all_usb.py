#!/usr/bin/env python3

import sys, os, time, glob
from serial import Serial

s_baud = 115200

dev = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
hser = Serial(dev, s_baud, timeout=5, writeTimeout=5)

sers = []
#for s_port in glob.glob('/dev/serial/by-id/usb*Black_Sphere_Technologies*'):
for s_port in glob.glob('/dev/serial/by-id/usb*n800s_UProgLink*'):
	sers.append(Serial(s_port, s_baud, timeout=5, writeTimeout=5))
#	print(s_port, s_baud)

time.sleep(1)

for ser in sers:
	ser.write(b'Ping\n')
	ser.flush()
	print('received: %s' % hser.readline())

hser.write(b'Pong\n')
hser.flush()

for i,ser in enumerate(sers):
	print(b'%d: %s' % (i, ser.readline()))


