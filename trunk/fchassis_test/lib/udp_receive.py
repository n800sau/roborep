#!/usr/bin/env python

from socket import *
import sys, os, shutil, time
import select


def udp_receive(port=9999):
	host="0.0.0.0"
	s = socket(AF_INET,SOCK_DGRAM)
	s.bind((host,port))

	addr = (host,port)
	buf=1024


	fname,addr = s.recvfrom(buf)

	print "Receiving File:",fname.strip()

	f = open(fname.strip(),'wb')

	s.settimeout(2)
	t = time.time()
	data,addr = s.recvfrom(buf)
	try:
		while(data):
			f.write(data)
			data,addr = s.recvfrom(buf)
	except timeout:
		f.write(data)
		f.close()
		s.close()
		print "File Downloaded for %.2f secs" % (time.time() - t)
		shutil.move(fname, os.path.join(os.path.expanduser('~/public_html/udp_receive'), os.path.basename(fname)))

if __name__ == '__main__':

	port = 9999

	udp_receive(port)
