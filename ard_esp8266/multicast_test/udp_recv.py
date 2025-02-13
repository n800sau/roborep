#!/usr/bin/env python

import sys
import datetime
import socket
import struct
import json
import time
import traceback

MCAST_GRP = '239.0.0.57'
MCAST_PORT = 12345

def dbprint(text):
    print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

def make_sock():

	i = 1
	while True:

		try:
			sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
			sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			sock.bind(('', MCAST_PORT))  # use MCAST_GRP instead of '' to listen only
                             # to MCAST_GRP, not all groups on MCAST_PORT
			mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

			sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
			break

		except:
			traceback.print_exc()
			time.sleep(5)
			dbprint('Attempt %d to bind' % i)
			i += 1

	return sock

if __name__ == '__main__':

	sock = make_sock()

	while True:
		try:
			dbprint('Waiting...')
			data = sock.recv(1000)
			try:
				dbprint('%s' % data)
			except ValueError, e:
				dbprint('%s: %s' % (e, data))
				continue
		except Exception, e:
			if isinstance(e, KeyboardInterrupt):
				raise
			traceback.print_exc()
