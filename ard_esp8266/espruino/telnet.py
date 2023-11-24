#!/usr/bin/env python

import sys, os
import telnetlib

HOST = "192.168.1.167"

tn = telnetlib.Telnet(HOST)

#tn.read_until(">")
#tn.write('print("Hello");\n')

sf = open("init.js")

while True:
	line = tn.read_until("\n", 1).strip()
	if line:
		if line in ('>', ':'):
			sline = sf.readline()
			if not sline:
				break
			tn.write(sline.strip() + '\r')
		else:
			print line

