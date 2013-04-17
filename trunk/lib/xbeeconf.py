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

		return reply

if __name__ == '__main__':
	import sys
	print ("*" * 20) + "\nConfiguring xbee..."

#pi SH,SL =  13 A2 00  40 92 D7 A0
#HP SH,SL =  13 A2 00  40 92 D7 19

#	cmdlist = ['ID', 'CH', 'MY', 'DL', 'DH', 'AP']

	for port,dh,dl in (
			('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A601EONT-if00-port0', '13A200', '4092D7A0'), 
			('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A601EM1A-if00-port0', '13A200', '4092D719')):
		print "\n"
		if os.path.exists(port):
			print 'Path %s:' % port

			cmdlist = ['ID', 'DH', 'DL', 'MY', 'SH', 'SL']
#			cmdlist = ['ID3332', 'DL1234', 'MY5678', 'D02', 'D12', 'IR14', 'IT5',]
#			cmdlist = ['BD', 'CH', 'MY', 'ID', 'ID1111', 'SH', 'SL', 'DH', 'DL', 'DH%s' % dh, 'DL%s' % dl, 'NI', 'ND', 'WR', 'AS']
#			cmdlist = ['RE']
			xbee = XBee(port, 115200);

			if not xbee.startCommandMode():
				print "\tCould not enter command mode"
				sys.exit(1)
			for cmd in cmdlist:
				print "\t" + cmd + ': ' + str(xbee.sendCommand(cmd))
			xbee.close()
		else:
			print 'Path %s does not exist' % port

	print "Configuring xbee finished\n" + ("*" * 20)
