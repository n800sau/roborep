#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import json
import time
import datetime
import math
import signal
import redis
import struct
import traceback
import threading
import Queue
from PyMata.pymata import PyMata
import pids

import logging
logging.basicConfig(filename='fserver_bmp.log',level=logging.DEBUG)


COUNT_PER_REV = 20.0
WHEEL_DIAMETER = 0.065
BASELINE = 0.14

ENC_STEP = WHEEL_DIAMETER * math.pi / COUNT_PER_REV


REDIS_CHANNEL = 'command'
SENSORS_SHOW_PERIOD = 1

MIN_PWR = 70
MAX_PWR = 255

DEV='/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

LEFT_MOTOR_1 = 6
LEFT_MOTOR_2 = 5

RIGHT_MOTOR_1 = 10
RIGHT_MOTOR_2 = 9

# encoder pins
ENCODER_L = 2
ENCODER_R = 3

# Indices into callback return data list
DEVICE = 0
PIN = 1
DATA = 2

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

def Fi2c_factory(name, board):

	def get_i2c_device(klass, addr):
		return Device(board, addr)

	rs = type(name, (object,), {})
	rs.get_i2c_device = classmethod(get_i2c_device)
	return rs

class Device:

	def __init__(self, board, addr):
		self.board = board
		self._address = addr

	def writeRaw8(self, value):
		"""Write an 8-bit value on the bus (without register)."""
		value = value & 0xFF
		self.board.i2c_write(self._address, value)

	def write8(self, register, value):
		"""Write an 8-bit value to the specified register."""
		value = value & 0xFF
		self.board.i2c_write(self._address, register, value)

	def write16(self, register, value):
		"""Write a 16-bit value to the specified register."""
		value = value & 0xFFFF
		self.board.i2c_write(self._address, register, value & 0xFF, value << 8)

	def writeList(self, register, data):
		"""Write bytes to the specified register."""
		self.board.i2c_write(self._address, register, *data)

	def readList(self, register, length):
		"""Read a length number of bytes from the specified register.  Results
		will be returned as a bytearray."""
		self.board.i2c_read(self._address, register, length, self.board.I2C_READ)
		time.sleep(0.2)
		rs = self.board.i2c_get_read_data(self._address)
		print >>sys.stderr, "data=%s" % rs
		return rs[1:] if rs else rs

	def readU8(self, register):
		"""Read an unsigned byte from the specified register."""
		return self.readList(register, 1)[0]

	def readS8(self, register):
		"""Read a signed byte from the specified register."""
		result = self.readU8(register)
		if result > 127:
			result -= 256
		return result

	def readU16(self, register, little_endian=True):
		"""Read an unsigned 16-bit value from the specified register, with the
		specified endianness (default little endian, or least significant byte
		first)."""
		rs = self.readList(register, 2)
		if rs:
			if little_endian:
				rs = (rs[1]<<8) | rs[0]
			else:
				rs = (rs[0]<<8) | rs[1]
		return rs


	def readS16(self, register, little_endian=True):
		"""Read a signed 16-bit value from the specified register, with the
		specified endianness (default little endian, or least significant byte
		first)."""
		result = self.readU16(register, little_endian)
		if result > 32767:
			result -= 65536
		return result

	def readU16LE(self, register):
		"""Read an unsigned 16-bit value from the specified register, in little
		endian byte order."""
		return self.readU16(register, little_endian=True)

	def readU16BE(self, register):
		"""Read an unsigned 16-bit value from the specified register, in big
		endian byte order."""
		return self.readU16(register, little_endian=False)

	def readS16LE(self, register):
		"""Read a signed 16-bit value from the specified register, in little
		endian byte order."""
		return self.readS16(register, little_endian=True)

	def readS16BE(self, register):
		"""Read a signed 16-bit value from the specified register, in big
		endian byte order."""
		return self.readS16(register, little_endian=False)


class fserver(pids.Pid):

	def __init__(self, s_dev, **config):
		self.keep_heading = None
		self.maxAccX = 0
		self.last_heading = 0
		self.r = redis.Redis()

		super(fserver, self).__init__(**config)
		self.debug = True
		self.left_dir = None
		self.right_dir = None
		self.target_steps = 0
		self.q = Queue.Queue()
		self.t = threading.Thread(target=redis_subscriber, args = (self.q,))
		self.t.setDaemon(True)
		self.t.start()


		self.range(-50.0, 50.0)
		self.tune(.8,.1,.1)

		self.enc_data = {
			ENCODER_L: {
				'name': 'left',
				'lvl': 0,
				'dir': 0,
				'tick_time': time.time(),
				'count': 0,
				'pwr': 0,
			},
			ENCODER_R: {
				'name': 'right',
				'lvl': 0,
				'dir': 0,
				'tick_time': time.time(),
				'count': 0,
				'pwr': 0,
			},
		}

		# Create a PyMata instance
		#self.board = PyMata(s_dev)
		self.board = PyMata(s_dev, verbose=False)

		signal.signal(signal.SIGINT, self.signal_handler)

		self.Fi2c = Fi2c_factory('Fi2c', self.board)

		# configure firmata for i2c on an UNO
		self.board.i2c_config(0, self.board.ANALOG, 4, 5)

		self.board.set_pin_mode(ENCODER_L, self.board.INPUT, self.board.DIGITAL)
		# Arm the digital latch to detect when the button is pressed
		self.arm_latch(ENCODER_L, True)

		self.board.set_pin_mode(ENCODER_R, self.board.INPUT, self.board.DIGITAL)
		# Arm the digital latch to detect when the button is pressed
		self.arm_latch(ENCODER_R, True)

	def __exit__(self, type, value, traceback):
		# close the interface down cleanly
		self.board.close()

	def signal_handler(self, sig, frame):
		print('You pressed Ctrl+C!!!!')
		if self.board is not None:
			self.board.reset()
		sys.exit(0)

	def dbprint(self, text, force=False):
		if self.debug or force:
			print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

	def measure(self):
		compass = self.get_last('hmc5883l', 2)
		if compass:
			self.last_heading = compass['heading_degrees']
		return self.last_heading

	def reset_counters(self):
		self.enc_data[ENCODER_L]['count'] = 0
		self.enc_data[ENCODER_R]['count'] = 0

	def steps_counted(self):
		return max(abs(self.enc_data[ENCODER_L]['count']), abs(self.enc_data[ENCODER_R]['count']))

	def adjust_pwr(self, pwr):
		self.dbprint('output=%f' % self.get())

	def right_move(self, direct, pwr):
		if direct:
			self.board.set_pin_mode(RIGHT_MOTOR_1, self.board.PWM, self.board.DIGITAL)
			self.board.set_pin_mode(RIGHT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
			self.board.analog_write(RIGHT_MOTOR_1, pwr)
			self.board.digital_write(RIGHT_MOTOR_2, 0)
		else:
			self.board.set_pin_mode(RIGHT_MOTOR_2, self.board.PWM, self.board.DIGITAL)
			self.board.set_pin_mode(RIGHT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
			self.board.analog_write(RIGHT_MOTOR_2, pwr)
			self.board.digital_write(RIGHT_MOTOR_1, 0)
		self.enc_data[ENCODER_R]['pwr'] = pwr
		if pwr > 0:
			self.right_dir = direct
			self.enc_data[ENCODER_R]['dir'] = direct

	def left_move(self, direct, pwr):
		if direct:
			self.board.set_pin_mode(LEFT_MOTOR_1, self.board.PWM, self.board.DIGITAL)
			self.board.set_pin_mode(LEFT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
			self.board.analog_write(LEFT_MOTOR_1, pwr)
			self.board.digital_write(LEFT_MOTOR_2, 0)
		else:
			self.board.set_pin_mode(LEFT_MOTOR_2, self.board.PWM, self.board.DIGITAL)
			self.board.set_pin_mode(LEFT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
			self.board.analog_write(LEFT_MOTOR_2, pwr)
			self.board.digital_write(LEFT_MOTOR_1, 0)
		self.enc_data[ENCODER_L]['pwr'] = pwr
		if pwr > 0:
			self.left_dir = direct
			self.enc_data[ENCODER_L]['dir'] = direct

	def arm_latch(self, pin, high):
		self.board.set_digital_latch(pin, self.board.DIGITAL_LATCH_HIGH if high else self.board.DIGITAL_LATCH_LOW, lambda data, pin=pin: self.cb_encoder(data, pin))

	def cb_encoder(self, data, pin):
		if data[2] != self.enc_data[pin]['lvl'] and data[2]:
			t = time.time()
			dt = t-self.enc_data[pin]['tick_time']
			step = (1 if self.enc_data[pin]['dir'] else -1)
			self.enc_data[pin]['count'] += step
			self.dbprint("pin: %d (%s), dt:%s, v:%.2f, cnt:%d" % (pin, self.enc_data[pin]['name'], dt, step * ENC_STEP / dt, self.enc_data[pin]['count']))
			self.enc_data[pin]['tick_time'] = t
		self.enc_data[pin]['lvl'] = data[2]
		self.arm_latch(pin, not data[2])
		if self.target_steps < abs(self.enc_data[pin]['count']) and self.enc_data[pin]['pwr'] > 0:
			if pin == ENCODER_L:
				self.left_move(0, 0)
			elif pin == ENCODER_R:
				self.right_move(0, 0)
			self.dbprint('%s stopped' % self.enc_data[pin]['name'])

	def is_stopped(self):
		return self.enc_data[ENCODER_L]['pwr'] == 0 and self.enc_data[ENCODER_R]['pwr'] == 0

	def stop(self):
		self.keep_heading = None
		if not self.is_stopped():
			self.left_move(0, 0)
			self.right_move(0, 0)
			self.dbprint('both stopped')

	def execute_cmd(self, cmd):
		self.dbprint('execute %s' % cmd)
		command = cmd['command']
		params = cmd.get('params', {})
		if command == 'stop':
			self.stop()
		else:
			pwr = params.get('power', 50)
			if pwr > 0:
				pwr = int(min(100, pwr) / 100. * (MAX_PWR - MIN_PWR) + MIN_PWR)
			self.target_steps = params.get('steps', 0)
			if self.target_steps > 0:
				self.reset_counters()
			self.dbprint('pwr: %d' % pwr)
			if command in ('mv_fwd', 'mv_back'):
				self.keep_heading = None
				compass = self.get_last('hmc5883l', 2)
				if compass:
					self.keep_heading = compass['heading_degrees']
					self.last_step_time = time.time()
					self.set(self.keep_heading)
				if command == 'mv_fwd':
					self.right_move(1, pwr)
					self.left_move(1, pwr)
				else:
					self.right_move(0, pwr)
					self.left_move(0, pwr)
			elif command == 't_left':
				self.right_move(1, pwr)
				self.left_move(0, pwr)
			elif command == 't_right':
				self.right_move(0, pwr)
				self.left_move(1, pwr)

	def get_last(self, sname, no_older_than=None):
		rs = self.r.lrange('%s.js.obj' % sname, 0, 0)
		if rs:
			rs = json.loads(rs[0])
			if not no_older_than is None:
#			print float(rs['timestamp']) + no_older_than, time.time()
				if float(rs['timestamp']) + no_older_than < time.time():
					rs = None
		return rs

	def cmd_show_sensors(self):
		r = {}
		try:
			compass = self.get_last('hmc5883l', 2)
			if compass:
				self.last_heading = compass['heading_degrees']
				r['heading'] = compass['heading_degrees']
			r['Rcount'] = self.enc_data[ENCODER_R]['count']
			r['Rpower'] = self.enc_data[ENCODER_R]['pwr']
			r['Lcount'] = self.enc_data[ENCODER_L]['count']
			r['Lpower'] = self.enc_data[ENCODER_L]['pwr']
			bmp085 = self.get_last('bmp085', 5)
			if bmp085:
				r['T'] = bmp085['temperature']
			adxl345 = self.get_last('adxl345', 2)
			if adxl345:
				if self.maxAccX < abs(adxl345['scaled']['x']):
					self.maxAccX = abs(adxl345['scaled']['x'])
			r['acc_x_max'] = self.maxAccX
		except:
			traceback.print_exc()
		self.r.lpush('sensors', json.dumps(r))
		self.r.ltrim('sensors', 0, 1000)


	def check_rcommands(self):
		rs = None
		try:
			rs = self.q.get(False)
		except Queue.Empty:
			pass
		return rs

	def run(self):
		t = time.time()
		while True:
			rcmd = self.check_rcommands()
			if rcmd:
#				self.dbprint('chan=%s' % rcmd[0])
#				self.dbprint('cmd=%s' % rcmd[1])
				self.execute_cmd(rcmd[1])
			else:
				tdiff = time.time() - t
				if tdiff > SENSORS_SHOW_PERIOD:
					t += tdiff
					self.cmd_show_sensors()
				else:
					time.sleep(0.1)
			if not self.keep_heading is None:
				if t-self.last_step_time > 0:
					compass = self.get_last('hmc5883l', 2)
					if compass:
						self.last_heading = compass['heading_degrees']
					self.step(dt=t-self.last_step_time, input=self.last_heading)
					adj_pwr = self.get()
					if abs(adj_pwr) >= 1:
						self.adjust_pwr(adj_pwr)
					self.last_step_time = t

if __name__ == '__main__':

	cfgobj = configure(os.path.dirname(__file__))

	cfg = cfgobj.as_dict('serial')
	s_port = cfg['serial_port']

	c = fserver(s_port)
	c.run()
