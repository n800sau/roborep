#!/usr/bin/env python

import os
from serial import Serial
from time import sleep

class XBee:
	CC = '+'
	GT = 1.1

	def __init__(self, port, baud):
		self.ser = Serial(port, baud, timeout=5, writeTimeout=5);

	def close(self):
		self.endCommandMode()
		self.ser.close()

	def write(self):
		self.sendCommand('WR')

	def startCommandMode(self):
		self.emptyBuffer()
		self.ser.write(self.CC + self.CC + self.CC)
		sleep(self.GT)
		return self.getReply() == 'OK'

	def emptyBuffer(self):
		#print("Empting buffer: {}".format(self.ser.read(self.ser.inWaiting())))
		self.ser.read(self.ser.inWaiting())

	def endCommandMode(self):
		return self.sendCommand('CN') == 'OK'

	def sendCommand(self, cmd):
		print 'cmd:', cmd
		self.emptyBuffer()
		self.ser.write('AT' + cmd + '\r')
		if cmd == "AS":
			sleep(5)
		return self.getReply()

	def getReply(self):
		count = 0
		reply = ''
		while True:
			char = self.ser.read()
			if char == '\r':
				break
			if len(char) == 0:
				return None
			reply += char

		print 'reply:', reply
		return reply

if __name__ == '__main__':
	import sys

	cmdlist = ['ID', 'CH', 'MY', 'DL', 'DH', 'AP']

	for port in ('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A601EONT-if00-port0', '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A601EM1A-if00-port0'):
		if os.path.exists(port):

			xbee = XBee(port, 9600);

			if not xbee.startCommandMode():
				print("Could not enter command mode")
				sys.exit(1)
			for cmd in cmdlist:
				print(cmd + ': ' + str(xbee.sendCommand(cmd)))
			xbee.close()
			break


