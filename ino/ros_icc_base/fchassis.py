#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import json
import time
import struct
import traceback
import pycmds
from bin2uno_inf import bin2uno_inf
from serial import Serial

SENSORS_SHOW_PERIOD = 1

class fchassis(bin2uno_inf):

	def __init__(self, s_port, s_baud):
		bin2uno_inf.__init__(self, pycmds.MAGIC_BYTE)
		self.debug = True
		self.wait4sensors = False
		self.s_port = s_port
		self.s_baud = s_baud
		self.ser = Serial(self.s_port, self.s_baud, timeout=5, writeTimeout=5);
		self.dbprint('Connect %s at %d' % (self.s_port, self.s_baud))

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
		for i in range(4):
			data = self.read_bin()
			self.dbprint('Data=%s' % data)
			if data:
				if data['cmd'] == pycmds.R_OK_0:
					self.dbprint('Ok')
					break
				elif data['cmd'] == pycmds.R_ERROR_0:
					self.dbprint('Error')
					break
			time.sleep(0.1)

	def cmd_state(self):
		self.wait4sensors = True
		self.debug = False
		rs = {}
		try:
			self.send_command(pycmds.C_STATE)
			for i in range(5):
				data = self.read_bin()
				if data:
					self.dbprint('Data=%s' % data)
					if 'cmd' in data:
						if data['cmd'] == pycmds.R_DIST_1F:
							rs['dist'] = data['vals'][0]
						elif data['cmd'] == pycmds.R_VOLTS_1F:
							rs['V'] = data['vals'][0]
						elif data['cmd'] == pycmds.R_MCOUNTS_2F:
							rs['Lcount'] = data['vals'][0]
							rs['Rcount'] = data['vals'][1]
						elif data['cmd'] == pycmds.R_MDIST_2F:
							rs['Ldist'] = data['vals'][0]
							rs['Rdist'] = data['vals'][1]
						elif data['cmd'] == pycmds.R_END:
							break
				else:
					time.sleep(0.001)
		except Exception, e:
			self.dbprint(traceback.format_exc())
			if isinstance(e, KeyboardInterrupt):
				raise
		finally:
			self.debug = True
			self.wait4sensors = False
		self.dbprint(json.dumps(rs, indent=4))
		return rs

	def flush(self):
		if self.ser.inWaiting() > 0:
			self.spin()
			data = self.read_bin()
			if data:
				self.dbprint('Received BIN: %s' % data)
			else:
				self.dbprint('no BIN replied')

	def translate_cmd(self, cmd):
		self.dbprint('execute %s' % cmd)
		command = cmd['command']
		params = cmd.get('params', {})
		s2b = {
			'stop': pycmds.C_MSTOP,
		}
		bcmd = s2b.get(command, None)
		if bcmd:
			self.dbprint('send %s' % command)
			self.send_command(bcmd, struct.pack('BB', params.get('power', 255), params.get('steps', 1)))
			reply = self.read_bin()
			if reply and reply['cmd'] == pycmds.R_OK_0:
				self.dbprint('Ok')

	def cmd_mstop(self):
		self.send_command(pycmds.C_MSTOP)
		self.wait_reply()

	def cmd_mboth(self, lpwm, lfwd, rpwm, rfwd):
		self.send_command(pycmds.C_MBOTH, struct.pack('BBBB', lpwm, lfwd, rpwm, rfwd))
		self.wait_reply()

	def run(self):
		data = ''
		t = time.time()
		while True:
			if not self.wait4sensors:
				tdiff = time.time() - t
				if tdiff > SENSORS_SHOW_PERIOD:
					t += tdiff
					self.cmd_state()
				self.flush()
			else:
				time.sleep(0.01)

if __name__ == '__main__':

	cfgobj = configure(os.path.dirname(__file__))

	cfg = cfgobj.as_dict('serial')
	s_port = cfg['serial_port']
	s_rate = int(cfg.get('baud_rate', 115200))

	c = fchassis(s_port, s_rate)
#	c.run()
	time.sleep(2)
	c.cmd_state()
	# move forward
	fwd = False
	c.cmd_mboth(50, fwd, 50, fwd)
	time.sleep(0.5)
	c.cmd_mstop()
	c.flush()
	c.cmd_state()
	time.sleep(1)
	c.cmd_state()

