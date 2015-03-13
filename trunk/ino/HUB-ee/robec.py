#!/usr/bin/env python

import sys
import socket
import select
import errno
import json

MSGLEN = 100

class robec:

	def __init__(self, sock=None):
		if sock is None:
			self.sock = socket.socket(
				socket.AF_INET, socket.SOCK_STREAM)
		else:
			self.sock = sock
		self.sf = self.sock.makefile()

	def connect(self, host, port):
		self.sock.connect((host, port))
		self.sock.setblocking(0)

	def run(self):
		json_read = False
		data = ''
		while True:
#			print 'waiting...'
			ready2read, ready2write, in_error = select.select([self.sock, sys.stdin], [], [])
			for s in ready2read:
				if s == self.sock:
#					print 'reading sock ...'
					while True:
						try:
							l = self.sf.readline()
						except socket.error:
							break
#						print 'Line:', l
						if l.startswith('JSON:'):
							json_read = True
							data = l[5:]
						elif json_read and l.strip() == '.':
							# json ready to use
							json_read = False
							try:
								jdata = json.loads(data)
								print jdata
							except Exception, e:
								print e
								print 'Received: "%s"' % data
							finally:
								data = ''
						elif json_read:
							data += l
						else:
							print l
				elif s == sys.stdin:
#					print 'reading stdin...'
					msg = sys.stdin.readline()
					self.sock.sendall(msg)
#					sys.stdout.write("Sent\n")
#					sys.stdout.flush()

if __name__ == '__main__':

	c = robec()
	c.connect('192.168.1.96', 23)
	c.run()
