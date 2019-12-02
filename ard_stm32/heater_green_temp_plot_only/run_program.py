#!/usr/bin/env python3

import sys, os, time, json
import serial, math

exec(open("vars.sh").read())

s_port = DEV
s_baud = 115200

ser = serial.Serial(s_port, s_baud, writeTimeout=5, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
			timeout=1, xonxoff=0, rtscts=0)
# Toggle DTR to reset STM32
ser.setDTR(True)
# set BOOT0
ser.setRTS(True)
time.sleep(1)
# toss any data already received, see
# http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
ser.flushInput()
ser.setDTR(False)
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
			't': 96,
			'tm': 60,
		},
		{
			't': 55,
			'tm': 60,
		},
		{
			't': 72,
			'tm': 60,
		},
		{
			't': 40,
			'tm': 30,
		},
		{
			't': 20,
			'tm': 1,
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
		temp_reached = False
		start_temp = None
		while time.time() < current_start_ts + item['tm']:
			line = ser.readline().strip()
			if line:
				print(ts(), line)
				if line.startswith(b'T '):
					t = float(line.split(b' ')[1])
					if start_temp is None:
						start_temp = t
					dt = abs(item['t'] - t)
					if dt < 1 and not temp_reached:
						dtm = time.time() - current_start_ts
						temp_reached = True
						print(ts(), 'Reached temp %d in %.2f sec (%.2f per sec)' % (item['t'], dtm, abs(start_temp-item['t'])/dtm))
			else:
#				print('sleep')
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

