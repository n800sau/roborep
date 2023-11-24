#!/usr/bin/env python

from serial import Serial
import sys, time, datetime

def read_line(ser):
  count = 0
  reply = '' 
  while True:
    char = ser.read() 
    if char == '\r': 
      break
    if len(char) == 0: 
      return ''
    reply += char

  return reply 

def dbprint(text):
	print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

ser = Serial('/dev/ttyUSB0', 115200, timeout=5, writeTimeout=5)

finished = False
t_last = time.time()
i = 0
while not finished:
	l = read_line(ser)
	l = l.strip();
	dbprint('stage 1: %s' % l)
	if l == 'Ready':
		t_last = time.time()
		if len(sys.argv) > 1:
			if sys.argv[1] == '1':
				ser.write('!')
			elif sys.argv[1] == '2':
				ser.write('@')
			elif sys.argv[1] == '3':
				ser.write('#')
		time.sleep(0.1)
		ser.write('c')
		ser.flush()
		while not finished:
			l = read_line(ser).strip();
			dbprint('stage 2: %s' % l)
			if l == 'Proceed':
				sf = file('firmware.hex')
				while not finished:
					l = read_line(ser).strip();
					dbprint('stage 3: %s' % l)
					if l =='Give line':
						t_last = time.time()
						hl = sf.readline()
						dbprint('line %s' % hl)
						if hl:
							ser.write(hl)
						else:
							finished = True
							dbprint('Finished')
							break
					elif l == 'Stopped':
						t_last = time.time()
						finished = True
						dbprint('Finished')
						break
			if time.time() - t_last > 10:
				raise Exception('timeout 2')
		break
	if time.time() - t_last > 10:
		raise Exception('timeout 1')
