#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, signal
import rospy
from time import time, sleep
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Imu

from twisted.internet import reactor, defer
from twisted.internet.serialport import SerialPort
from twisted.protocols import basic
from twisted.internet.task import LoopingCall

DEVICE = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0'
BAUD = 57600

# Ordered this way to minimize wait time.
rospy.init_node('imu_ir', anonymous = True)
soundhandle = SoundClient()

class Echo(basic.LineReceiver):

	def connectionMade(self):
		self.pub = rospy.Publisher('/oculus2wd/imu', Imu)
		rospy.loginfo("Serial port connected")
#		self.transport.write('>>> ')

	def lineReceived(self, line):
		line = line.strip().split('\t')
		if line[0] == 'quat':
			imu = Imu()
			imu.orientation.x = line[1]
			imu.orientation.y = line[2]
			imu.orientation.z = line[3]
			imu.orientation.w = line[4]
			self.pub.publish(imu)
		print line
#		self.sendLine('Echo: ' + line)
#		self.transport.write('>>> ')

def sayBye():
	print "bye bye."

SerialPort(Echo(), DEVICE, reactor, baudrate=BAUD)

#loopObj = LoopingCall(rospy.spinOnce)
#loopObj.start(0.01, now=True)

reactor.addSystemEventTrigger('during', 'shutdown', sayBye)

#class Token(object):
#	def __init__(self):
#		self.running = True

#token = Token()

def customHandler(signum, stackframe):
#	print "Got signal: %s" % signum
#	token.running = False                # to stop my code
	reactor.callFromThread(reactor.stop) # to stop twisted code when in the reactor loop

signal.signal(signal.SIGINT, customHandler)

#def oneKeyboardInterruptHandler(failure):
#	failure.trap(KeyboardInterrupt)
#	print "INTERRUPTING one(): KeyboardInterrupt"
#	reactor.stop()

#d = defer.Deferred()
#d.addErrback(oneKeyboardInterruptHandler)

reactor.run()
