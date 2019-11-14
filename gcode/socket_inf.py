#!/usr/bin/env python

import socket, time, sys, telnetlib

HOST = 'opiplus2e.local'
PORT = 4000

tn = telnetlib.Telnet(HOST, PORT, 1)
tn.write("\n")
print 'Waiting...'
print tn.read_until('\r')

while True:
	line = sys.stdin.readline()
	if not line.strip():
		break
	print line
	tn.write(line)
	print tn.read_until('\r')

print 'Finished'
