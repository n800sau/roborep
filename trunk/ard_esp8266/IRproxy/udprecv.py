#!/usr/bin/env python

import socket
import struct
import json
import subprocess

MCAST_GRP = '239.0.0.57'
MCAST_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))  # use MCAST_GRP instead of '' to listen only
                             # to MCAST_GRP, not all groups on MCAST_PORT
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
	try:
		data = sock.recv(10240)
		data = json.loads(data)
		text = 'encoding:%s, value:0x%4.4x' % (data['encoding'], data['ircode'])
		print text
		subprocess.call(['espeak', text])
	except Exception, e:
		if isinstance(e, KeyboardInterrupt):
			raise
		print 'Error', e

