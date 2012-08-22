#!/usr/bin/env python
from time import time, strftime
import math

class Ro:

	def __init__(self):
		#x - to north
		self.x = 0
		#y - to west
		self.y = 0
		#direction
		self.heading = 0
		#rotation (degreese per second)
		self.rspeed = 0
		#head speed (m per second)
		self.speed = 0
		self.ctime = self.starttime = time()
		#step (sec)
		self.step = 0.0001

	def dtime(self):
		return time() - self.ctime

	def process(self):
		t = time()
		while self.ctime < t:
			self.heading = (self.heading + self.step * self.rspeed) % 360
			distance = self.speed * self.step
			hrad = math.radians(self.heading)
			self.x += math.cos(hrad) * distance
			self.y += math.sin(hrad) * distance
			self.ctime += self.step

	def run(self):
		data = None
		while data != 'x':
			data = raw_input("%d>> " % (time() - self.starttime))
			self.process()
			data = data.split()
			if data:
				if data[0] == 'q':
					self.rspeed -= float(data[1] if data[1:] else 1)
				elif data[0] == 'e':
					self.rspeed += float(data[1] if data[1:] else 1)
				if data[0] == 'w':
					self.speed += float(data[1] if data[1:] else 1)
				elif data[0] == 's':
					self.speed -= float(data[1] if data[1:] else 1)
				elif data[0] == 'z':
					self.rspeed = 0
					self.speed = 0
			print "(%d: %d, %d, sp:%g rsp:%g >>%d)" % (time() - self.starttime, self.x, self.y, self.speed, self.rspeed, self.heading)

acc_list = []
def test_accelerator():
	x = x_ax()
	y = y_ax()
	z = z_ax()
	if abs(x) > 1000 or abs(y) > 1000 or abs(z) > 1000:
		stop()
	remove noise
	smooth list
	position of max in list
	maximum of list in range

Ro().run()
