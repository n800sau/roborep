#!/usr/bin/env python

import socket, time, sys, telnetlib

HOST = 'opiplus2e.local'
PORT = 4000

tn = telnetlib.Telnet(HOST, PORT, 1)
tn.write("\n")
print 'Waiting...'
print tn.read_until('\r')

tn.write("M4\n")

print tn.read_until('\r')
print tn.read_all()
