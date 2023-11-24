#!/usr/bin/env python3

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))
import tty, termios
import argparse
from serial import Serial

parser = argparse.ArgumentParser(description='Programmer control')
parser.add_argument('--mode', choices=['esp', 'stm'], default='esp')
args = parser.parse_args()

exec(open("vars.sh").read())

s_port = DEV
s_baud = 115200

ser = Serial(s_port, s_baud, timeout=5, writeTimeout=5, dsrdtr=None)
#print s_port, s_baud

ser.write(b'?')
ser.flush()

def loop():
	while True:
		print('Input %s:(p,w,r,i,x)' % args.mode, end='')
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
			if args.mode == 'stm':
				if val.upper() == 'W':
					val = 'p'
				elif val.upper() == 'P':
					val = 'w'
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
