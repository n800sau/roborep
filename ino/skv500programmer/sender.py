#!/usr/bin/env python

from serial import Serial
import sys, time

def read_line(ser):
  count = 0
  reply = '' 
  while True:
    char = ser.read() 
    if char == '\r': 
      break
    if len(char) == 0: 
      return None
    reply += char

  return reply 

ser = Serial('/dev/ttyUSB0', 115200, timeout=5, writeTimeout=5)

finished = False
t_start = time.time()
i = 0
while not finished:
	l = read_line(ser).strip();
	print 'stage 1:', l
	if l == 'Ready':
		ser.write('c')
		ser.flush()
		while not finished:
			l = read_line(ser).strip();
			print 'stage 2:', l
			if l == 'Proceed':
				sf = file('firmware.hex')
				while not finished:
					l = read_line(ser).strip();
					print 'stage 3:', l
					if l =='Give line':
						hl = sf.readline()
						if hl:
							ser.write(hl)
						else:
							finished = True
							print 'Finished'
							break
				break
			if time.time() - t_start > 10:
				print 'timeout'
				break
		break
	if time.time() - t_start > 10:
		print 'timeout'
		break
