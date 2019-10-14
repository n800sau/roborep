import sys, os, time, json
import serial, math

exec(open("vars.sh").read())

def init_serial(reset=True):
	started = True

	s_port = DEV
	s_baud = 115200

	ser = serial.Serial(s_port, s_baud, writeTimeout=5, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
			timeout=1, xonxoff=0, rtscts=0)
	if reset:
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
		started = False
		for i in range(5):
			line = ser.readline().strip()
			if line:
				print('Line:', line)
			if line == b'Ready':
				started = True
				break
	return ser if started else None

def ts():
	return time.strftime('%d/%m/%Y %H:%M:%S')

def send_cmd(ser, cmd):
	ser.flush()
	ser.write(cmd + b'\n')
	ser.flush()
	print(ts(), 'Sent:', cmd)

def read_print_all(ser):
	while True:
		line = ser.readline().strip()
		if line:
			print(line)
