#!/usr/bin/env python

from serial import Serial
import json, sys

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

leds = [' ', ' ']

ser = Serial('/dev/ttyO1', 57600, timeout=5, writeTimeout=5)

i = 0
while True:
	try:
		data = json.loads(read_line(ser))
		if data['led'] in (0, 1):
			leds[data['led']] = 'O' if data['on'] else 'o'
			sys.stdout.write("\r%-10d|%s|" % (i, ''.join(leds)))
			sys.stdout.flush()
			i += 1
	except ValueError:
		continue
