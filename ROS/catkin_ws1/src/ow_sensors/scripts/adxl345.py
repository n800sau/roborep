#!/usr/bin/env python

# ADXL345 Python library for Raspberry Pi 
#
# author:  Jonathan Williamson
# license: BSD, see LICENSE.txt included in this package
# 
# This is a Raspberry Pi Python implementation to help you get started with
# the Adafruit Triple Axis ADXL345 breakout board:
# http://shop.pimoroni.com/products/adafruit-triple-axis-accelerometer

# requires python library smbus-cffi

import smbus
from time import sleep

# select the correct i2c bus for this revision of Raspberry Pi
revision = ([l[12:-1] for l in open('/proc/cpuinfo','r').readlines() if l[:8]=="Revision"]+['0000'])[0]

class ADXL345:

	# ADXL345 constants
	EARTH_GRAVITY_MS2	= 9.80665
	SCALE_MULTIPLIER	= 0.004

	DATA_FORMAT			= 0x31
	BW_RATE				= 0x2C
	POWER_CTL			= 0x2D

	BW_RATE_1600HZ		= 0x0F
	BW_RATE_800HZ		= 0x0E
	BW_RATE_400HZ		= 0x0D
	BW_RATE_200HZ		= 0x0C
	BW_RATE_100HZ		= 0x0B
	BW_RATE_50HZ		= 0x0A
	BW_RATE_25HZ		= 0x09

	RANGE_2G			= 0x00
	RANGE_4G			= 0x01
	RANGE_8G			= 0x02
	RANGE_16G			= 0x03

	MEASURE				= 0x08
	AXES_DATA			= 0x32

	REG_INT_ENABLE		= 0x2E
	REG_INT_MAP			= 0x2F

	REG_FIFO_CTL		= 0x38
	REG_FIFO_STATUS		= 0x39

	FIFO_BYPASS			= 0
	FIFO_FIFO			= 1
	FIFO_STREAM			= 2
	FIFO_TRIGGER		= 3

	FIFO_TRIGGER_INT1	= 0
	FIFO_TRIGGER_INT2	= 1

	def __init__(self, address = 0x53):
		self.bus = smbus.SMBus(1 if int(revision, 16) >= 4 else 0)
		self.address = address
		self.setBandwidthRate(self.BW_RATE_25HZ)
		self.setRange(self.RANGE_2G)
		self.enableMeasurement()

	def enableMeasurement(self):
		self.bus.write_byte_data(self.address, self.POWER_CTL, self.MEASURE)

	def setBandwidthRate(self, rate_flag):
		self.bus.write_byte_data(self.address, self.BW_RATE, rate_flag)

	def setFIFOmode(self, mode, trigger_int, sample_bits):
		bits = ((mode & 3) << 6) | ((trigger_int & 1) << 5) | (sample_bits & 0x1f)
		self.bus.write_byte_data(self.address, self.REG_FIFO_CTL, bits)

	def getFIFOstatus(self):
		bits = self.bus.read_byte_data(self.address, self.REG_FIFO_STATUS)
		#print '%4.4X' % bits
		return {'triggered': bits & 0x80, 'entries': bits & 0x3f}

	# set the measurement range for 10-bit readings
	def setRange(self, range_flag):
		value = self.bus.read_byte_data(self.address, self.DATA_FORMAT)

		value &= ~0x0F;
		value |= range_flag;  
		value |= 0x08;

		self.bus.write_byte_data(self.address, self.DATA_FORMAT, value)
	
	# returns the current reading from the sensor for each axis
	#
	# parameter gforce:
	#	 False (default): result is returned in m/s^2
	#	 True			: result is returned in gs
	def getAxes(self, gforce = False):
		bytes = self.bus.read_i2c_block_data(self.address, self.AXES_DATA, 6)
		
		x = bytes[0] | (bytes[1] << 8)
		if(x & (1 << 16 - 1)):
			x = x - (1<<16)

		y = bytes[2] | (bytes[3] << 8)
		if(y & (1 << 16 - 1)):
			y = y - (1<<16)

		z = bytes[4] | (bytes[5] << 8)
		if(z & (1 << 16 - 1)):
			z = z - (1<<16)

		x = x * self.SCALE_MULTIPLIER 
		y = y * self.SCALE_MULTIPLIER
		z = z * self.SCALE_MULTIPLIER

		if gforce == False:
			x = x * self.EARTH_GRAVITY_MS2
			y = y * self.EARTH_GRAVITY_MS2
			z = z * self.EARTH_GRAVITY_MS2

		x = round(x, 4)
		y = round(y, 4)
		z = round(z, 4)

		return {"x": x, "y": y, "z": z}

	def getAxesList(self, gforce=False):
		rs = []
		st = self.getFIFOstatus()
		for i in range(st['entries']):
			rs.append(self.getAxes(gforce))
		return rs

if __name__ == "__main__":
	# if run directly we'll just create an instance of the class and output 
	# the current readings
	adxl345 = ADXL345(0x1d)
#	adxl345 = ADXL345()
	adxl345.setFIFOmode(adxl345.FIFO_BYPASS, adxl345.FIFO_TRIGGER_INT2, 2)
	adxl345.setFIFOmode(adxl345.FIFO_FIFO, adxl345.FIFO_TRIGGER_INT2, 2)

	sleep(0.03)
	print "ADXL345 on address 0x%x:" % (adxl345.address)
	st = adxl345.getFIFOstatus()
	print st
	for i in range(st['entries']):
		axes = adxl345.getAxes(True)
		print "%d:\n	  x = %.3fG" % ( i, axes['x'] )
#		print "	  y = %.3fG" % ( axes['y'] )
#		print "	  z = %.3fG" % ( axes['z'] )
