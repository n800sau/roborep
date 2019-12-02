#!/usr/bin/env python

import serial, time

def request(cmd):
	print cmd, ':',
	ser.write(cmd + '\x0d\x0a')
	while True:
		data = ser.read(1000)
		if len(data):
			print data,
		else:
			break
	print

DEV = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
#DEV = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'

ser = serial.Serial(DEV, 9600, timeout=0.1)
#request('AT')
#request('AT+DEFAULT')
#request('AT+RENEW')
#time.sleep(5)
#request('AT+ROLE0')
#time.sleep(5)
#request('AT+VERSION')
#request('AT+PIN888888')
#request('AT+PASS')
#request('AT+BAUD')
#request('AT+IBEA')
#request('AT+IBE0')
#request('AT+IBE1')
#request('AT+MARJ')
#request('AT+MINO')
request('AT+POWE4')
#request('AT+NOTI')
#request('AT+UUID0x3333')
request('AT+UUID')
#request('AT+CHAR')
#request('AT+ROLE')
#request('AT+LADDR')
#request('AT+ADDR')
#request('AT+IBEA1')
#request('AT+NAME')
request('AT+NAMENECO2')
#request('AT+NAME')
#request('AT+ROLE0')
#request('AT+INQ')
#request('AT+CONN1')

#request('AT+HELP')

request('AT+MARJ0x1234')
request('AT+MINO0xFA02')
#request('AT+ADVI5')
#request('AT+NOTI1')
request('AT+IBEA1')
request('AT+RESET')
#request('AT+PWRM1')
#request('AT+START')

#while True:
#	data = ser.read(1)
#	if len(data):
#		print data,
#	else:
#		break

ser.close()
