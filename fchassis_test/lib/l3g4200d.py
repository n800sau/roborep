#!/usr/bin/env python

import math
import smbus
from time import *
import struct

class l3g4200:

	WHO_AM_I      = 0x0F

	CTRL_REG1     = 0x20
	CTRL_REG2     = 0x21
	CTRL_REG3     = 0x22
	CTRL_REG4     = 0x23
	CTRL_REG5     = 0x24
	REFERENCE     = 0x25
	OUT_TEMP      = 0x26
	STATUS_REG    = 0x27

	OUT_X_L       = 0x28
	OUT_X_H       = 0x29
	OUT_Y_L       = 0x2A
	OUT_Y_H       = 0x2B
	OUT_Z_L       = 0x2C
	OUT_Z_H       = 0x2D

	FIFO_CTRL_REG = 0x2E
	FIFO_SRC_REG  = 0x2F

	INT1_CFG      = 0x30
	INT1_SRC      = 0x31
	INT1_THS_XH   = 0x32
	INT1_THS_XL   = 0x33
	INT1_THS_YH   = 0x34
	INT1_THS_YL   = 0x35
	INT1_THS_ZH   = 0x36
	INT1_THS_ZL   = 0x37
	INT1_DURATION = 0x38

	def __init__(self, port, address=0x69):
		self.bus = smbus.SMBus(port)
		self.address = address
		self.setDefaults()

	def setDefaults(self, scale=1):
		#From  Jim Lindblom of Sparkfun's code

		# Enable x, y, z and turn off power down:
		self.bus.write_byte_data(self.address, self.CTRL_REG1, 0xf)

		# If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
		self.bus.write_byte_data(self.address, self.CTRL_REG2, 0)

		# Configure CTRL_REG3 to generate data ready interrupt on INT2
		# No interrupts used on INT1, if you'd like to configure INT1
		# or INT2 otherwise, consult the datasheet:
		self.bus.write_byte_data(self.address, self.CTRL_REG3, 0x8)

		# CTRL_REG4 controls the full-scale range, among other things:

		if scale == 250:
			self.bus.write_byte_data(self.address, self.CTRL_REG4, 0)
		elif scale == 500:
			self.bus.write_byte_data(self.address, self.CTRL_REG4, 0x10)
		else:
			self.bus.write_byte_data(self.address, self.CTRL_REG4, 0x30)

		# CTRL_REG5 controls high-pass filtering of outputs, use it
		# if you'd like:
		self.bus.write_byte_data(self.address, self.CTRL_REG5, 0)


	def getWhoAmI(self):
		return self.bus.read_byte_data(self.address, self.WHO_AM_I)

	def getDieTemperature(self):
		temp = self.bus.read_word_data(self.address, self.OUT_TEMP | 0x80)
		return temp
#		return round(35 + (temp + 13200.) / 280, 2)

	def getAxes(self):
		data = self.bus.read_i2c_block_data(self.address, self.OUT_X_L | 0x80, 6)
		data = struct.pack('BBBBBB', *data)
		x, y, z = struct.unpack('<hhh', data)
#		x = struct.unpack('>h', struct.pack('>H', self.bus.read_word_data(self.address, self.OUT_X_L | 0x80)))
#		y = struct.unpack('>h', struct.pack('>H', self.bus.read_word_data(self.address, self.OUT_Y_L | 0x80)))
#		z = struct.unpack('>h', struct.pack('>H', self.bus.read_word_data(self.address, self.OUT_Z_L | 0x80)))
		return (x, y, z)

	def getDegPerSecAxes(self):
		(gyro_x, gyro_y, gyro_z) = self.getAxes()
		return (gyro_x / 14.375, gyro_y / 14.375, gyro_z / 14.375)

if __name__ == "__main__":
	import sys
	import time
	# http://magnetic-declination.com/Great%20Britain%20(UK)/Harrogate#
	gyro = l3g4200(1)
	print 'Who:%s' % gyro.getWhoAmI()
	while True:
		st = gyro.bus.read_byte_data(gyro.address, gyro.STATUS_REG)
		if st:
			x,y,z = gyro.getDegPerSecAxes()
			sys.stdout.write("\rT:%d, %g %g %g          " % (gyro.getDieTemperature(), x, y, z))
			sys.stdout.flush()
#			time.sleep(0.5)



