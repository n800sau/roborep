#!/usr/bin/env python

import sys
import socket
import select
import errno
import json
from StringIO import StringIO

# end of json marker
EOJ = "\n.\n"

class robec:

	def __init__(self, sock=None):
		if sock is None:
			self.sock = socket.socket(
				socket.AF_INET, socket.SOCK_STREAM)
		else:
			self.sock = sock
		self.recv_data = ''
		self.dbg_data = ''

	def connect(self, host, port):
		self.sock.connect((host, port))
		self.sock.setblocking(0)

	def send_json(self, jsondict):
		self.sock.sendall(json.dumps(jsondict) + EOJ)

	def send_command(self, command, **kwds):
		kwds['command'] = command
		return self.send_json(kwds)

	def read_dbg(self):
		rs = self.dbg_data
		self.dbg_data = ''
		return rs

	def read_line(self):
		rs = ''
#		print 'REMAIN:', self.recv_data
		self.recv_data = self.recv_data.replace('\x0d', '\x0a')
		self.recv_data = self.recv_data.replace('\x0a\x0a', '\x0a')
		nldex = self.recv_data.find('\x0a')
		if nldex >= 0:
			rs = self.recv_data[:nldex + 1]
			self.recv_data = self.recv_data[nldex + 1:]
		return rs

	def read_json(self):
		rs = None
		data = ''
		json_read = False
		while rs is None:
			self.spin()
			l = self.read_line()
			if l:
				if l.startswith('JSON:'):
					json_read = True
					data = l[5:]
				elif json_read and l.strip() == '.':
					# json ready to use
					json_read = False
					try:
						rs = json.loads(data)
					except Exception, e:
						self.dbg_data += str(e)
						self.dbg_data += 'Received: "%s"' % data
						rs = ''
					break
				elif json_read:
					data += l
				else:
					self.dbg_data += l
			elif not json_read:
				break
		return rs

	def spin(self):
		while True:
			try:
				data = self.sock.recv(1024)
				if data:
					self.recv_data += data
				else:
					break
			except socket.error, e:
				if e.errno == 11:
#					if self.recv_data:
#						print 'len=', len(self.recv_data)
					break
				else:
					raise

	def run(self):
		json_read = False
		data = ''
		while True:
#			print 'waiting...'
			ready2read, ready2write, in_error = select.select([self.sock, sys.stdin], [], [])
			for s in ready2read:
				if s == self.sock:
#					print 'reading'
					self.spin()
					jdata = self.read_json()
					if jdata:
						print jdata
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