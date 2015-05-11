#!/usr/bin/env python

import socket
import sys
import time
import select
import json
import traceback
import redis
import pycmds

from robec import robec

RDATALIST = '6050'
TIMEOUT = 0.001

class d2r(robec):

	def issuer(self):
		return 'd2rcmd'

	def process_bin(self, data):
		if data['cmd'] == pycmds.R_ACC_3F:
#			print >>sys.__stderr__, time.time(), "received"
			data['t'] = time.time()
			self.r.lpush(RDATALIST, json.dumps(data))
			self.r.ltrim(RDATALIST, 0, 99)

	def before_run(self):
		self.t = time.time()
		self.send_at('USEC')
		self.send_at('BAUD 115200')

	def idle(self):
		if time.time() - self.t > TIMEOUT:
			self.send_command(pycmds.C_STATE)
#			print >>sys.__stderr__, time.time(), "send"
			self.t = time.time()

if __name__ == '__main__':

	c = d2r()
	c.connect('192.168.1.96', 23)
	c.run()
