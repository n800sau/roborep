#!/usr/bin/env python

import sys
import socket
import select
import array
import errno
import json
import time
import redis
import threading
import Queue
from StringIO import StringIO
import pycmds

def redis_subscriber(q):
	r = redis.Redis()
	s = r.pubsub()
	lsnr = s.listen()
	s.subscribe('command')
	print >>sys.stderr, 'subscribed', lsnr.next()
	while True:
		try:
			command = lsnr.next()
			print >>sys.stderr, 'command=', command
			if command['type'] == 'message':
				q.put((command['channel'], command['data']))
		except Exception, e:
			print e

class robec:

	def __init__(self, sock=None):
		self.debug = True
		if sock is None:
			self.sock = socket.socket(
				socket.AF_INET, socket.SOCK_STREAM)
		else:
			self.sock = sock

		self.q = Queue.Queue()
		self.t = threading.Thread(target=redis_subscriber, args = (self.q,))
		self.t.setDaemon(True)
		self.t.start()

		self.recv_data = ''
		self.dbg_data = ''
		self.r = redis.Redis()
		self.r.delete('vectors')

	def dbprint(self, text):
		if self.debug:
			print >>sys.__stderr__, text

	def connect(self, host, port):
		self.sock.connect((host, port))
		self.sock.setblocking(0)

	def send_line(self, line):
		self.dbprint('Sending %s' % line)
		self.sock.sendall(line + '\n')

	def send_bytes(self, bytes):
		self.dbprint('Sending %s' % list(bytes))
		self.sock.sendall(bytes)

	def crc8(self, bytes):
		crc = 0
		for extract in list(bytes):
			extract = ord(extract)
			for i in range(8):
				sm = (crc ^ extract) & 0x01
				crc >>= 1
				if sm:
					crc ^= 0x8C
				extract >>= 1
		return crc

	def send_command(self, command, databytes=None):
		self.dbprint('send_command')
		packet = '\x85' + chr(command)
		if databytes is None:
			packet += chr(0)
		else:
			packet += chr(len(databytes)) + databytes
		self.send_bytes(packet + chr(self.crc8(packet[1:])))
		return self.read_bin()

	def send_at(self, command):
		return self.send_line('+++AT' + command)

	def read_dbg(self):
		rs = self.dbg_data
		self.dbg_data = ''
		return rs

	def read_line(self):
		rs = ''
#		self.dbprint( 'REMAIN: %s' % self.recv_data)
		self.recv_data = self.recv_data.replace('\x0d', '\x0a')
		self.recv_data = self.recv_data.replace('\x0a\x0a', '\x0a')
		nldex = self.recv_data.find('\x0a')
		if nldex >= 0:
			rs = self.recv_data[:nldex + 1]
			self.recv_data = self.recv_data[nldex + 1:]
		return rs

	def read_json(self):
		t = time.time()
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
						state = json.loads(data)
						self.dbprint('Received %s' % rs)
						if rs.get('vects', None):
							for v in rs['vects']:
								self.r.rpush('vectors', json.dumps(v))
						if 'vects' in state:
							del state['vects']
						self.r.rpush('state', json.dumps(state))
					except Exception, e:
						self.dbprint(e)
						self.dbprint('Received bad json: %s\n' % data)
						self.dbg_data += str(e)
						self.dbg_data += 'Received: "%s"' % data
						rs = ''
						if isinstance(e, KeyboardInterrupt):
							raise
					break
				elif json_read:
					data += l
				else:
					self.dbprint('Line: %s\n' % l)
					self.dbg_data += l
			elif not json_read:
				break
			elif t + 1 < time.time():
				raise Exception('timeout')
		return rs

	def parametrize(self, data):
		data['millis'] = array.array('L', data['data'][:4])[0]
		data['vals'] = [v for v in array.array('f', data['data'][4:])]
		del data['data']
		return data

	def read_bin(self):
		self.dbprint("read_bin")
		t = time.time()
		rs = None
		bin_read = False
		while rs is None:
			self.spin()
			if bin_read:
				l = len(self.recv_data)
				if l > 3:
					cmd,sz = array.array('B', self.recv_data[1:3])
					# magic byte , cmd, size, (size bytes), crc = 4 + size
					if l >= sz + 4:
						self.recv_data[:sz+4]
						# test crc
						c_crc = self.crc8(self.recv_data[:sz+3])
						r_crc = ord(self.recv_data[sz+3])
						if c_crc == r_crc:
							# ok
							rs = self.parametrize({'cmd': cmd, 'data': self.recv_data[3:sz+3]})
						else:
							self.dbprint("crc error %s != %s" % (c_crc, r_crc))
#						self.dbprint(': %s\n' % self.recv_data[0])
						# discard magic byte and continue
						self.recv_data = self.recv_data[1:]
						bin_read = False
			else:
#				if not self.recv_data:
#					break
				if self.recv_data:
					try:
						ss = self.recv_data.index(chr(pycmds.MAGIC_BYTE))
						bin_read = True
						discarded = self.recv_data[:ss]
						self.recv_data = self.recv_data[ss:]
					except ValueError:
						discarded = self.recv_data
						self.recv_data = ''
					if discarded:
						for line in discarded.split('\r'):
							line = line.strip()
#						if line.startswith('MSG:'):
#							self.dbprint(': %s\n' % line)
						self.dbprint('Discarded: %s\n' % discarded)
						self.dbg_data += discarded
			if t + 1 < time.time():
				raise Exception('timeout')
		return rs

	def spin(self):
		while True:
			try:
				data = self.sock.recv(1024)
				if data:
#					self.dbprint('len=%d' % len(data))
#					self.dbprint('data=%s' % data)
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

	def check_rcommands(self):
		rs = None
		try:
			rs = self.q.get(False)
#				print '####', d, '###'
		except Queue.Empty:
			pass
		return rs

	def run(self):
		json_read = False
		data = ''
		while True:
#			print 'waiting...'
			self.check_rcommands()
			ready2read, ready2write, in_error = select.select([self.sock, sys.stdin], [], [], 1)
			for s in ready2read:
				if s == self.sock:
#					print 'reading'
					self.spin()
					data = self.read_bin()
					if data:
						self.dbprint('Received BIN: %s' % data)
					else:
						self.dbprint('no BIN replied')
#					jdata = self.read_json()
#					if jdata:
#						self.dbprint('Received JSON: %s' % jdata)
#					else:
#						self.dbprint('no JSON replied')
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
