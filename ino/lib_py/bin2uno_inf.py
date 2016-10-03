import sys
import json
import time
import datetime
import array

def buf2hex(buf):
	line = []
	for c in array.array('B', buf):
		line.append('%2.2X' % c)
	return ' '.join(line)


class bin2uno_inf(object):

	def __init__(self, magic_byte):
		self.debug = False
		self.magic_byte = magic_byte
		self.recv_data = ''
		self.dbg_data = ''

	# should write data
	def send2ino(self, s):
		pass

	# should read data block and return it
	# if there is no data should return ''
	def recv(self):
		return ''

	def dbprint(self, text, force=False):
		if self.debug or force:
			print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

	def send_bytes(self, bytes):
#		self.dbprint('Sending %s' % buf2hex(bytes))
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
#		self.dbprint('send_command')
		packet = chr(self.magic_byte) + chr(command)
		if databytes is None:
			packet += chr(0)
		else:
			packet += chr(len(databytes)) + databytes
		self.send_bytes(packet + chr(self.crc8(packet[1:])))

	def read_dbg(self):
		rs = self.dbg_data
		self.dbg_data = ''
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
#		self.dbprint("read_bin")
		t = time.time()
		rs = None
		bin_read = False
		while rs is None:
			if t + 1 < time.time():
				# timeout
				rs = None
				break
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
						r_crc = ord(self.recv_data[sz+3]) if len(self.recv_data) > sz+3 else 0
#						self.dbprint('%s vs %s' % (c_crc, r_crc))
						if c_crc == r_crc:
							# ok
							rs = self.parametrize({'cmd': cmd, 'data': self.recv_data[3:sz+3]})
							self.recv_data = self.recv_data[sz+4:]
#							self.dbprint('rs = %s' % rs)
						else:
							self.dbprint("crc error %s != %s" % (c_crc, r_crc))
#						self.dbprint(': %2.2X\n' % ord(self.recv_data[0]))
							# discard magic byte and continue
							self.recv_data = self.recv_data[1:]
						bin_read = False
			else:
#				if not self.recv_data:
#					break
				if self.recv_data:
					try:
						ss = self.recv_data.index(chr(self.magic_byte))
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
#						self.dbprint('Discarded: %s\n' % ' '.join([('%2.2x' % ord(c)) for c in discarded]))
			#			self.dbprint('Discarded: %s\n' % discarded)
						self.dbg_data += discarded
#		self.dbprint('returns %s' % rs)
		return rs

if __name__ == '__main__':

	a = bin2uno_inf()
	# should raise exception "timeout"
	a.send_command(1)

