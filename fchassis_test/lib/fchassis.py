#!/usr/bin/env python

import sys, os
import time
import datetime
import math
import signal
import traceback
from PyMata.pymata import PyMata

COUNT_PER_REV = 20.0
WHEEL_DIAMETER = 0.065
BASELINE = 0.14

ENC_STEP = WHEEL_DIAMETER * math.pi / COUNT_PER_REV

MIN_PWR = 70
MAX_PWR = 255

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

class fchassis:

	def __init__(self, s_dev):

		self.debug = True
		self.left_dir = None
		self.right_dir = None

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

		# configure firmata for i2c on an UNO
		self.board.i2c_config(0, self.board.ANALOG, 4, 5)

		self.board.set_pin_mode(ENCODER_L, self.board.INPUT, self.board.DIGITAL)
		# Arm the digital latch to detect when the button is pressed
		self.arm_latch(ENCODER_L, True)

		self.board.set_pin_mode(ENCODER_R, self.board.INPUT, self.board.DIGITAL)
		# Arm the digital latch to detect when the button is pressed
		self.arm_latch(ENCODER_R, True)

	def __enter__(self):
		return self

	def __exit__(self, type, value, traceback):
		# close the interface down cleanly
		print 'Exiting...'
		try:
			self.board.close()
		except:
			pass
		time.sleep(1)

	def signal_handler(self, sig, frame):
		print('You pressed Ctrl+C!!!!')
		if self.board is not None:
			self.board.reset()
		sys.exit(0)

	def dbprint(self, text, force=False):
		if self.debug or force:
			print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

	def reset_counters(self):
		self.enc_data[ENCODER_L]['count'] = 0
		self.enc_data[ENCODER_R]['count'] = 0

	def steps_counted(self):
		return max(abs(self.enc_data[ENCODER_L]['count']), abs(self.enc_data[ENCODER_R]['count']))

	def adjust_pwr(self, pwr):
		self.dbprint('output=%f' % self.get())

	def lpwr(self, pwr):
		return int(min(100, pwr) / 100. * (MAX_PWR - MIN_PWR) + MIN_PWR)

	def right_move(self, direct, pwr):
		if pwr == 0:
				self.board.set_pin_mode(RIGHT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
				self.board.set_pin_mode(RIGHT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
				self.board.digital_write(RIGHT_MOTOR_1, 0)
				self.board.digital_write(RIGHT_MOTOR_2, 0)
#				self.dbprint('right PWM stopped')
		else:
			lpwr = self.lpwr(pwr)
			if direct:
				self.board.set_pin_mode(RIGHT_MOTOR_1, self.board.PWM, self.board.DIGITAL)
				self.board.set_pin_mode(RIGHT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
				self.board.analog_write(RIGHT_MOTOR_1, lpwr)
				self.board.digital_write(RIGHT_MOTOR_2, 0)
			else:
				self.board.set_pin_mode(RIGHT_MOTOR_2, self.board.PWM, self.board.DIGITAL)
				self.board.set_pin_mode(RIGHT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
				self.board.analog_write(RIGHT_MOTOR_2, lpwr)
				self.board.digital_write(RIGHT_MOTOR_1, 0)
		self.enc_data[ENCODER_R]['pwr'] = pwr
		if pwr > 0:
			self.right_dir = direct
			self.enc_data[ENCODER_R]['dir'] = direct

	def left_move(self, direct, pwr):
		if pwr == 0:
				self.board.set_pin_mode(LEFT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
				self.board.set_pin_mode(LEFT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
				self.board.digital_write(LEFT_MOTOR_1, 0)
				self.board.digital_write(LEFT_MOTOR_2, 0)
#				self.dbprint('left PWM stopped')
		else:
			lpwr = self.lpwr(pwr)
			if direct:
				self.board.set_pin_mode(LEFT_MOTOR_1, self.board.PWM, self.board.DIGITAL)
				self.board.set_pin_mode(LEFT_MOTOR_2, self.board.OUTPUT, self.board.DIGITAL)
				self.board.analog_write(LEFT_MOTOR_1, lpwr)
				self.board.digital_write(LEFT_MOTOR_2, 0)
			else:
				self.board.set_pin_mode(LEFT_MOTOR_2, self.board.PWM, self.board.DIGITAL)
				self.board.set_pin_mode(LEFT_MOTOR_1, self.board.OUTPUT, self.board.DIGITAL)
				self.board.analog_write(LEFT_MOTOR_2, lpwr)
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

	def is_stopped(self):
		return self.enc_data[ENCODER_L]['pwr'] == 0 and self.enc_data[ENCODER_R]['pwr'] == 0

	def stop(self):
		self.keep_heading = None
		if not self.is_stopped():
			self.left_move(0, 0)
			self.right_move(0, 0)
			self.dbprint('both stopped')

if __name__ == '__main__':

	s_port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH00ZTCM-if00-port0'

	with fchassis(s_port) as c:

		try:
			c.left_move(False, 50)
			c.right_move(True, 50)
			time.sleep(1)
		finally:
			c.stop()
