#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import json
import time
import serial
import redis
import struct
import traceback
import threading
import Queue
import pycmds
from bin2uno_inf import bin2uno_inf
from serial import Serial

REDIS_CHANNEL = 'command'
SENSORS_SHOW_PERIOD = 1

def redis_subscriber(q):
	r = redis.Redis()
	s = r.pubsub()
	lsnr = s.listen()
	s.subscribe(REDIS_CHANNEL)
	print >>sys.stderr, 'subscribed to', REDIS_CHANNEL
	while True:
		try:
			command = lsnr.next()
#			print >>sys.stderr, 'command=', command
			if command['type'] == 'message':
				q.put((command['channel'], json.loads(command['data'])))
		except Exception, e:
			print e

class irserver(bin2uno_inf):

	def __init__(self, s_port, s_baud):
		bin2uno_inf.__init__(self, pycmds.MAGIC_BYTE)
		self.debug = True
		self.wait4sensors = False
		self.q = Queue.Queue()
		self.t = threading.Thread(target=redis_subscriber, args = (self.q,))
		self.t.setDaemon(True)
		self.t.start()
		self.s_port = s_port
		self.s_baud = s_baud
		self.ser = Serial(self.s_port, self.s_baud, timeout=5, writeTimeout=5);
		self.dbprint('Connect %s at %d' % (self.s_port, self.s_baud))
		self.ser.open()
		# to skip setup output
		time.sleep(4)
		self.r = redis.Redis()
		self.r.delete('xy')
		self.hit_time = None

	def send2ino(self, s):
		self.ser.write(s)
		self.ser.flush()

	def recv(self):
		cnt = self.ser.inWaiting()
		if cnt > 0:
			self.dbprint('%d bytes available' % cnt)
			rs = self.ser.read(cnt)
		else:
			rs = ''
		return rs

	def check_rcommands(self):
		rs = None
		try:
			rs = self.q.get(False)
		except Queue.Empty:
			pass
		return rs

	def wait_reply(self):
		for i in range(100):
			data = self.read_bin()
			self.dbprint('Data=%s' % data)
			if data and data['cmd'] == pycmds.R_OK_0:
				self.dbprint('Ok')
				break
			time.sleep(0.1)

	def cmd_show_sensors(self):
		self.wait4sensors = True
		self.debug = False
		try:
			r = {
				'V': None,
				'heading': None,
				'dist': None,
				'acc_x_max': None,
				'Lcoef': None,
				'Rcoef': None,
				'Lcount': None,
				'Rcount': None,
				'T': None,
				'vects': [],
			}
			self.send_command(pycmds.C_STATE)
			processed = []
			for i in range(100):
				data = self.read_bin()
				if data:
					self.dbprint('Data=%s' % data)
					if 'cmd' in data:
						processed.append(str(data['cmd']))
					if data['cmd'] == pycmds.R_HIT_1F:
						if data['vals'][0]:
							self.hit_time = time.time()
						elif self.hit_time:
							if time.time() - self.hit_time > 5:
								self.hit_time = None
					elif data['cmd'] == pycmds.R_ACC_3F:
						r['acc_x'],r['acc_y'],r['acc_z'] = data['vals']
						self.r.lpush('accvec', json.dumps({'millis': data['millis'], 't': time.time(), 'x': r['acc_x'], 'y': r['acc_y'], 'z': r['acc_z']}))
						self.r.ltrim('accvec', 0, 1000)
					elif data['cmd'] == pycmds.R_ACCMAX_3F:
						r['acc_x_max'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_HEADING_1F:
						r['heading'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_DISTANCE_1F:
						r['dist'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_MCOUNTS_2F:
						r['Lcount'],r['Rcount'] = data['vals']
					elif data['cmd'] == pycmds.R_POWER_2F:
						r['Lpower'],r['Rpower'] = data['vals']
					elif data['cmd'] == pycmds.R_MCOEF_2F:
						r['Lcoef'],r['Rcoef'] = data['vals']
					elif data['cmd'] == pycmds.R_TEMPERATURE_1F:
						r['T'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_VOLTS_1F:
						r['V'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_VECTOR_3F:
						r['vects'].append({'lC': data['vals'][0], 'rC': data['vals'][1], 'h': data['vals'][2]})
					elif data['cmd'] == pycmds.R_END:
#						self.dbprint('Done:%s' % (' '.join(processed)), True)
						break
				else:
					time.sleep(0.001)
			self.r.lpush('sensors', json.dumps(r))
			self.r.ltrim('sensors', 0, 1000)
		except Exception, e:
			self.dbprint(traceback.format_exc())
			if isinstance(e, KeyboardInterrupt):
				raise
		finally:
			self.debug = True
			self.wait4sensors = False

	def translate_cmd(self, cmd):
		self.dbprint('execute %s' % cmd)
		command = cmd['command']
		params = cmd.get('params', {})
		s2b = {
			'stop': pycmds.C_STOP,
			'mv_fwd': pycmds.C_FORWARD,
			'mv_back': pycmds.C_BACK,
			't_left': pycmds.C_TLEFT,
			't_right': pycmds.C_TRIGHT,
		}
		bcmd = s2b.get(command, None)
		if bcmd:
			self.dbprint('send %s' % command)
			self.send_command(bcmd, struct.pack('BB', params.get('power', 255), params.get('steps', 1)))
			reply = self.read_bin()
			if reply and reply['cmd'] == pycmds.R_OK_0:
				self.dbprint('Ok')

	def run(self):
		data = ''
		t = time.time()
		while True:
#			print 'waiting...'
			rcmd = self.check_rcommands()
			if rcmd:
#				print 'chan=', rcmd[0]
#				print 'cmd=', rcmd[1]
				self.translate_cmd(rcmd[1])
			elif not self.wait4sensors:
				tdiff = time.time() - t
				if tdiff > SENSORS_SHOW_PERIOD:
					t += tdiff
					self.cmd_show_sensors()
			if self.ser.inWaiting() > 0:
#				print 'reading'
				self.spin()
				data = self.read_bin()
				if data:
					self.dbprint('Received BIN: %s' % data)
				else:
					self.dbprint('no BIN replied')
			else:
				time.sleep(0.01)

if __name__ == '__main__':

	cfgobj = configure(os.path.dirname(__file__))

	cfg = cfgobj.as_dict('serial')
	s_port = cfg['serial_port']
	s_rate = int(cfg.get('baud_rate', 9600))

	c = irserver(s_port, s_rate)
	c.run()

