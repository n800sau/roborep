#!/usr/bin/env python

from hmc5883l import hmc5883l
from adxl345 import ADXL345
from l3g4200d import l3g4200
import redis, time, json
from utils import dbprint
from sensor_const import *

REDIS_LIMIT = 100

class Sensors:

	def __init__(self, adxl345_address=0x53):
		self.r = redis.Redis(db=2)

		self.adxl345 = ADXL345(adxl345_address)
		dbprint("ADXL345 on address 0x%x" % (self.adxl345.address))
		self.adxl345.setFIFOmode(self.adxl345.FIFO_BYPASS, self.adxl345.FIFO_TRIGGER_INT2, 2)
		self.adxl345.setFIFOmode(self.adxl345.FIFO_FIFO, self.adxl345.FIFO_TRIGGER_INT2, 2)

		self.hmc5883l = hmc5883l(gauss = 4.7, declination = (-2,5))
		dbprint("HMC5883L on address 0x%x" % (self.hmc5883l.address))

		self.l3g4200 = l3g4200(1)
		dbprint("L3G4200 on address 0x%x" % (self.l3g4200.address))

		self.r.delete(ACCEL_REDIS_QUEUE, COMPASS_REDIS_QUEUE, GYRO_REDIS_QUEUE, TEMPERATURE_REDIS_QUEUE)

	def process_adxl345(self):
		st = self.adxl345.getFIFOstatus()
		for i in range(st['entries']):
			axes = self.adxl345.getAxes(True)
			self.r.rpush(ACCEL_REDIS_QUEUE, json.dumps({'time': time.time(), 'type': 'axes', 'axes': axes}))
			self.r.ltrim(ACCEL_REDIS_QUEUE, -REDIS_LIMIT, -1)
#			dbprint("%d:\n  %s" % ( i, axes))

	def process_hmc5883l(self):
		heading = self.hmc5883l.heading()
		self.r.rpush(COMPASS_REDIS_QUEUE, json.dumps({'time': time.time(), 'type': 'heading', 'heading': heading}))
		self.r.ltrim(COMPASS_REDIS_QUEUE, -REDIS_LIMIT, -1)
#		dbprint("Heading: %g" % heading)

	def process_l3g4200(self):
		st = self.l3g4200.bus.read_byte_data(self.l3g4200.address, self.l3g4200.STATUS_REG)
		if st:
			degsec = self.l3g4200.getDegPerSecAxes()
			t = self.l3g4200.getDieTemperature()
			self.r.rpush(GYRO_REDIS_QUEUE, json.dumps({'time': time.time(), 'type': 'degsec', 'degsec': degsec}))
			self.r.ltrim(GYRO_REDIS_QUEUE, -REDIS_LIMIT, -1)
			self.r.rpush(TEMPERATURE_REDIS_QUEUE, json.dumps({'time': time.time(), 'type': 'temperature', 'temperature': t}))
			self.r.ltrim(TEMPERATURE_REDIS_QUEUE, -REDIS_LIMIT, -1)
#			dbprint("\rT:%d, %g %g %g          " % (t, degsec[0], degsec[1], degsec[2]))

if __name__ == "__main__":

	s = Sensors()

	while True:
		time.sleep(0.01)
		s.process_adxl345()
		time.sleep(0.01)
		s.process_hmc5883l()
		time.sleep(0.01)
		s.process_l3g4200()
