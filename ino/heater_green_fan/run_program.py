#!/usr/bin/env python

import sys, os, time, json
import serial

execfile("vars.sh")

s_port = DEV
s_baud = 115200

ser = serial.Serial(s_port, s_baud, writeTimeout=5, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
			timeout=1, xonxoff=0, rtscts=0)
# Toggle DTR to reset Arduino
ser.setDTR(False)
time.sleep(1)
# toss any data already received, see
# http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
ser.flushInput()
ser.setDTR(True)
#print s_port, s_baud

started = False
for i in range(5):
	line = ser.readline().strip()
	if line:
		print('Line:', line)
	if line == 'Ready':
		started = True
		break

if started:
	finished = False
	program = [
		{
			't': 30,
			'tm': 10,
		},
		{
			't': 10,
			'tm': 5,
		},
		{
			't': 30,
			'tm': 10,
		},
	]

	current_index = -1
	current_start_ts = None

	def run_item(item):
		print('Run:', json.dumps(item), time.strftime('%d/%m/%Y %H:%M:%S'))
		ser.write('T %d' % item['t'])
		ser.flush()
		current_start_ts = time.time()
		while time.time() < current_start_ts + item['tm']:
			print ser.readline()
			time.sleep(1)
		print('Stop:', json.dumps(item), time.strftime('%d/%m/%Y %H:%M:%S'))

	while not finished:
		if current_index < 0:
			current_index = 0
		if current_index >= len(program):
			finished = True
		else:
			run_item(program[current_index])
			current_index += 1


else:
	print('No Ready from device')

