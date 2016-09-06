#!/usr/bin/env python

from socket import *
import sys

def udp_send(host, port=9999, data_name='greeting', data='hello'):
	s = socket(AF_INET,SOCK_DGRAM)
	addr = (host,port)
	s.sendto(data_name, addr)
	while len(data):
		if(s.sendto(data[:1024],addr)):
			print "sending ..."
		data = data[1024:]
	s.close()
