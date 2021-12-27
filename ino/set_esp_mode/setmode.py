#!/usr/bin/env python3

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))
import tty, termios

from serial import Serial

exec(open("vars.sh").read())

s_port = DEV
s_baud = 115200

ser = Serial(s_port, s_baud, timeout=5, writeTimeout=5, dsrdtr=None)
#print s_port, s_baud

ser.write(b'?')
ser.flush()

def loop():
	while True:
		print('Input:(p,w,r,i,x)', end='')
		sys.stdout.flush()
		val = sys.stdin.read(1)
#		print('CHAR:', val)
		if val:
			print()
			if val.upper() == 'X':
				print('Exit')
				break
			if not val:
				val = "#"
			ser.write(val.encode('ascii'))
			ser.flush()

			for i in range(100):
				line = ser.readline().decode().strip()
				print(line)
				if line == 'end':
					break

tattr = termios.tcgetattr(sys.stdin)

try:
	tty.setcbreak(sys.stdin, termios.TCSANOW)

	for i in range(5):
		line = ser.readline().decode().strip()
		print(line)
		if line == 'Ready':
			loop()
			break
		else:
			time.sleep(1)

finally:
	termios.tcsetattr(sys.stdin, termios.TCSANOW, tattr)
