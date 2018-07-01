#!/usr/bin/env python
# -*- coding: utf-8 -*-
from nrf24 import NRF24
import time
import struct
import redis
import json

REDIS_LIST = 'l_gaz'

SENSOR_ADDRESS = [0x7d, 0x5a, 0x2e, 0x8b, 0x6f]
DATABASE_ADDRESS = [0x39, 0x02, 0x3e, 0x93, 0x99]

pipes = [DATABASE_ADDRESS, SENSOR_ADDRESS]

r = redis.Redis()

radio = NRF24()
radio.begin(2, 0, "P9_27", "P8_7") #Set CE and IRQ pins
#radio.begin(2, 0, "P9_27") #Set CE and IRQ pins
radio.reset()
time.sleep(2)

def setup_radio():

	#radio.setRetries(15,15)

	#radio.setPayloadSize(32)
	radio.setChannel(0x4C)
	radio.setDataRate(NRF24.BR_1MBPS)
	radio.setPALevel(NRF24.PA_HIGH)
	radio.enableDynamicPayloads()
	radio.setCRCLength(NRF24.CRC_16);
	#radio.disableCRC()
	radio.setRetries(10, 20)

	radio.setAutoAck(True)
#	print 'retries:', radio.getRetries()

	radio.openWritingPipe(pipes[0])
	radio.openReadingPipe(1, pipes[1])

setup_radio()
radio.stopListening()
radio.startListening()
radio.printDetails()

def time_print():
	print time.strftime('%H:%M:%S'),

pipe = [1]
notified = 0
while True:
#	print radio.testCarrier()
	if radio.available(pipe, False):
		notified = 0
#		time_print()

#		print 'Payload size:' , radio.getDynamicPayloadSize()
		recv_buffer = []
		radio.read(recv_buffer)
#		print pipe, recv_buffer

		load = struct.pack('B' * len(recv_buffer), *recv_buffer).strip()
		if load:
#			print load[:-1]
			t = time.time()
			data = dict([(v[0], v[1:]) for v in load[:-1].split(',')])
			data.update({
				'ts': int(t),
				'ts_str': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t)),
			})
			r.lpush(REDIS_LIST, json.dumps(data, ensure_ascii=True))
			print data
	else:
		if not notified:
			time_print()
			print 'Not available'
			notified = time.time()
		if notified + 30 < time.time():
			time_print()
			print 'Still not available. Radio reset.'
			radio.reset()
			time.sleep(1)
			setup_radio()
			radio.stopListening()
			radio.startListening()
			notified = time.time()
#			radio.flush_rx()
#			radio.flush_tx()
#	else:
#		print 'Waiting for irq'
#		irq_res = radio.irqWait()
#		print 'Stopped waiting for irq with result:', irq_res
