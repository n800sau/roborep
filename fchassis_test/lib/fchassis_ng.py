#!/usr/bin/env python

import sys, os, socket
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'ino', 'lib_py'))

if socket.gethostname() == 'orange':
	ino_path = 'orange_on_track'
elif socket.gethostname() == 'hubee':
	ino_path = 'ros_icc_base'
#	ino_path = 'orange_on_wheels'
else:
	raise Exception('Hostname %s not in list' % socket.gethostname())

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'ino', ino_path))

from conf_ino import configure
import json
import time
import struct
import traceback
import pycmds
from bin2uno_inf import bin2uno_inf
from serial import Serial

SENSORS_SHOW_PERIOD = 1

class fchassis_ng(bin2uno_inf):

	def __init__(self, debug=False):

		super(fchassis_ng, self).__init__(pycmds.MAGIC_BYTE)

		self.debug = debug

		self.wait4sensors = False
		self.distance = None
		self.voltage = None

		cfgobj = configure(os.path.join(os.path.dirname(__file__), '..', '..', 'ino', ino_path))

		cfg = cfgobj.as_dict('serial')
		self.s_port = cfg['serial_port']
		self.s_baud = int(cfg.get('baud_rate', 115200))

#		self.ser = Serial(self.s_port, self.s_baud, timeout=5, writeTimeout=5)
		self.ser = Serial(self.s_port, self.s_baud, timeout=5)
		self.dbprint('Connected %s at %d' % (self.s_port, self.s_baud))

		for i in range(50):
			self.send_command(pycmds.C_PING)
			if self.wait_reply():
				break
			time.sleep(0.1)


		self.last_tick = time.time()
		self.state = {
				'tick_time': self.last_tick,
				'lcount': 0,
				'rcount': 0,
				'lpwr': 0,
				'rpwr': 0,
				'ldist': 0,
				'rdist': 0,
				'v': 0,
				'sonar': -1,
		}

	def steps_counted(self):
		return max(abs(self.state['lcount']), abs(self.state['rcount']))

	def send2ino(self, s):
		self.ser.write(s)
		self.ser.flush()

	def recv(self):
		cnt = self.ser.inWaiting()
		if cnt > 0:
#			self.dbprint('%d bytes available' % cnt)
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
		rs = None
		for i in range(4):
			data = self.read_bin()
#			self.dbprint('Reply=%s' % data)
			if data:
				if data['cmd'] == pycmds.R_OK_0:
#					self.dbprint('Ok')
					rs = True
					break
				elif data['cmd'] == pycmds.R_ERROR_0:
					self.dbprint('Error')
					rs = False
					break
			time.sleep(0.1)
		return rs

	def update_state(self):
		self.wait4sensors = True
		old_debug = self.debug
#		self.debug = False
		rs = False
		try:
			self.send_command(pycmds.C_STATE)
			for i in range(10):
				data = self.read_bin()
				if data:
					self.dbprint('State data=%s' % data)
					if 'cmd' in data:
						rs = True
						if data['cmd'] == pycmds.R_DIST_1F:
							self.state['sonar'] = data['vals'][0]
						elif data['cmd'] == pycmds.R_IRDIST_1F:
							self.state['irdist'] = data['vals'][0]
						elif data['cmd'] == pycmds.R_VOLTS_1F:
							self.state['v'] = data['vals'][0]
						elif data['cmd'] == pycmds.R_MOTION_1F:
							self.state['motion'] = data['vals'][0]
						elif data['cmd'] == pycmds.R_MCOUNTS_2F:
							self.state['tick_time'] = time.time()
							self.state['lcount'] = data['vals'][0]
							self.state['rcount'] = data['vals'][1]
						elif data['cmd'] == pycmds.R_MCURRENT_2F:
							self.state['lcurrent'] = data['vals'][0]
							self.state['rcurrent'] = data['vals'][1]
						elif data['cmd'] == pycmds.R_MPOWER_2F:
							self.state['lpwr'] = data['vals'][0]
							self.state['rpwr'] = data['vals'][1]
						elif data['cmd'] == pycmds.R_MDIST_2F:
							self.dbprint(json.dumps(data, indent=4))
							self.state['ldist'] = data['vals'][0]
							self.state['rdist'] = data['vals'][1]
						elif data['cmd'] == pycmds.R_END:
							break
				else:
					time.sleep(0.01)
			if rs:
				self.dbprint(json.dumps(self.state, indent=4))
		except Exception, e:
			self.dbprint(traceback.format_exc())
			if isinstance(e, KeyboardInterrupt):
				raise
		finally:
			self.debug = old_debug
			self.wait4sensors = False
		return rs

	def flush(self):
		if self.ser.inWaiting() > 0:
			self.spin()
			data = self.read_bin()
			if data:
				self.dbprint('Received BIN: %s' % data)
			else:
				self.dbprint('no BIN replied')

	def cmd_mstop(self):
		self.send_command(pycmds.C_MSTOP)
		self.wait_reply()

	def cmd_reset_counters(self):
		self.send_command(pycmds.C_RESET_COUNTERS)
		self.wait_reply()
		self.state['tick_time'] = time.time()
		self.state['lcount'] = 0
		self.state['rcount'] = 0

	def cmd_mboth(self, lpwm, lfwd, rpwm, rfwd):
		self.send_command(pycmds.C_MBOTH, struct.pack('BBBB', min(255, lpwm), lfwd, min(255, rpwm), rfwd))
		self.wait_reply()

	def cmd_mleft(self, lpwm, lfwd):
		self.send_command(pycmds.C_MLEFT, struct.pack('BB', min(255, lpwm), lfwd))
		self.wait_reply()

	def cmd_mright(self, rpwm, rfwd):
		self.send_command(pycmds.C_MRIGHT, struct.pack('BB', min(255, rpwm), rfwd))
		self.wait_reply()

	def db_state(self):
		self.update_state()
		if self.state['tick_time'] > self.last_tick:
			self.last_tick = self.state['tick_time']
			self.dbprint("v:%.2f, lcnt:%d, rcnt:%d, dist:%s" % (
				self.state['v'], self.state['lcount'], self.state['rcount'], self.state['sonar']
			))

	def m2steps(self, m):
		return None

	def run(self):
		data = ''
		t = time.time()
		while True:
			if not self.wait4sensors:
				tdiff = time.time() - t
				if tdiff > SENSORS_SHOW_PERIOD:
					t += tdiff
					self.update_state()
				self.flush()
			else:
				time.sleep(0.01)

if __name__ == '__main__':

	c = fchassis_ng()
	c.run()
