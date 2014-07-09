#!/usr/bin/env python

import sys, os

from twisted.internet import reactor 
from twisted.internet.protocol import Factory 
from twisted.internet.protocol import Protocol
from twisted.internet.serialport import SerialPort
from twisted.python import log

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import libcommon_py as common

log.startLogging(sys.stdout)

#TCP clients connecting to me 
client_list = []

serial_list = []

class USBClient(Protocol):

	def __init__(self, network):
		self.network = network

	def connectionFailed(self):
		print "Connection Failed:", self
		reactor.stop()

	def connectionMade(self):
		serial_list.append(self)
		print 'Connected to serial'

	def dataReceived(self, data):
#		print "Data received", repr(data)
#		print "Data received! with %d bytes!" % len(data)
		for cli in client_list:
			cli.transport.write(data)

	def lineReceived(self, line):
		print "Line received", repr(line)

	def sendLine(self, cmd):
		print cmd
		self.transport.write(cmd + "\r\n")

	def outReceived(self, data):
		print "outReceived! with %d bytes!" % len(data)
		self.data = self.data + data

class CommandRx(Protocol):

	def connectionMade(self):
		print 'Connection received from tcp..'
		client_list.append(self)

	def dataReceived(self, data):
		print 'Command receive', repr(data)
		for usb in serial_list:
			usb.transport.write(data)

	def connectionLost(self, reason):
		print 'Connection lost', reason
		if self in client_list:
			print "Removing " + str(self)
			client_list.remove(self)

class CommandRxFactory(Factory):
	protocol = CommandRx

if __name__ == '__main__':

	cfgobj = configure(os.path.dirname(__file__))

	cfg = cfgobj.as_dict('serial')
	s_port = cfg['serial_port']
	s_rate = int(cfg.get('baud_rate', 9600))
	cfg = cfgobj.as_dict('serial2tcp')
	tcp_port = int(cfg.get('tcp_port', 9700))

	tcpfactory = CommandRxFactory()
	reactor.listenTCP(tcp_port, tcpfactory)
	SerialPort(USBClient(tcpfactory), s_port, reactor, baudrate=s_rate)
	reactor.run()
