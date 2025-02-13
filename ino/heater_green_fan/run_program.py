#!/usr/bin/env python3

import sys, os, time, json
import serial

exec(open("vars.sh").read())

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

def ts():
	return time.strftime('%d/%m/%Y %H:%M:%S')

started = False
for i in range(5):
	line = ser.readline().strip()
	if line:
		print('Line:', line)
	if line == b'Ready':
		started = True
		break

if started:
	finished = False
	program = [
		{
			't': 30,
			'tm': 30,
		},
		{
			't': 10,
			'tm': 20,
		},
		{
			't': 30,
			'tm': 10,
		},
	]

	current_index = -1
	current_start_ts = None

	def run_item(item):
		print(ts(), 'Run:', json.dumps(item))
		cmd = b'T %d\n' % item['t']
		ser.flush()
		ser.write(cmd)
		ser.flush()
		print(ts(), 'Sent:', cmd)
		current_start_ts = time.time()
		while time.time() < current_start_ts + item['tm']:
			line = ser.readline().strip()
			if line:
				print(ts(), line)
			else:
				print('sleep')
				time.sleep(0.1)
		print(ts(), 'Stop:', json.dumps(item))

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

