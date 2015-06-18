import sys
import json
import time

class bin2uno_inf:

	def __init__(self):
		self.debug = False
		self.recv_data = ''
		self.dbg_data = ''

	# should write data
	def send2ino(self, s):
		pass

	# should read data block and return it
	# if there is no data should return ''
	def recv(self):
		return ''


	def dbprint(self, text):
		if self.debug:
			print >>sys.__stderr__, text

	def send_line(self, line):
		self.dbprint('Sending %s' % line)
		self.send2ino(line + '\n')

	def send_bytes(self, bytes):
		self.dbprint('Sending %s' % list(bytes))
		self.send2ino(bytes)

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

	def parametrize(self, data):
		data['millis'] = array.array('L', data['data'][:4])[0]
		data['vals'] = [v for v in array.array('f', data['data'][4:])]
		del data['data']
		return data

	def spin(self):
		while True:
			data = self.recv()
			if data:
#				self.dbprint('len=%d' % len(data))
#				self.dbprint('data=%s' % data)
				self.recv_data += data
			else:
				break

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

if __name__ == '__main__':

	a = bin2uno_inf()
	# should raise exception "timeout"
	a.send_command(1)

