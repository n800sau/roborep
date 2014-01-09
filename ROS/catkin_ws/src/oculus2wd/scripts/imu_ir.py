#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, signal
import rospy
from time import time, sleep
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Imu
from oculus2wd.msg import drive_status, drive, arduino_power

from twisted.internet import reactor, defer
from twisted.internet.serialport import SerialPort
from twisted.protocols import basic
from twisted.internet.task import LoopingCall

DEVICE = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0'
BAUD = 57600

ARDUINO_ID = 'imu_ir'


# Ordered this way to minimize wait time.
rospy.init_node('imu_ir', anonymous = True)
soundhandle = SoundClient()

class Processor(basic.LineReceiver):

	def __init__(self, *args, **kwds):
		self.commands = {
			0x40BF906F: {'code': 'FORWARD', 'proc': self.do_forward},
			0x40BF807F: {'code': 'BACK', 'proc': self.do_back},
			0x40BF50AF: {'code': 'LEFT', 'proc': self.do_left},
			0x40BF10EF: {'code': 'RIGHT', 'proc': self.do_right},
			0x40BFD02F: {'code': 'OK', 'proc': self.do_stop},
			0xFFFFFFFF: {'code': 'REPEAT', 'proc': self.do_repeat},
		}
		self.pub = rospy.Publisher('/oculus2wd/imu', Imu)
		self.cmdpub = rospy.Publisher('/oculus_base_command', drive)
		self.pwrpub = rospy.Publisher('/oculus2wd/arduino_power', arduino_power)
		self.imu = None

	def connectionMade(self):
		rospy.loginfo("Serial port connected")
#		self.transport.write('>>> ')

	def lineReceived(self, line):
		line = line.strip()
		if line == 'START':
			self.imu = Imu()
		elif not self.imu is None:
			if line == 'END':
#				print 'IMU=', self.imu
				self.pub.publish(self.imu)
				self.imu = None
			else:
				line = line.split('\t')
				if line[0] == 'quat':
					self.imu.orientation.w = float(line[1])
					self.imu.orientation.x = float(line[2])
					self.imu.orientation.y = float(line[3])
					self.imu.orientation.z = float(line[4])
#				elif line[0] == 'euler':
#					self.imu.angular_velocity.x = float(line[1])
#					self.imu.angular_velocity.y = float(line[2])
#					self.imu.angular_velocity.z = float(line[3])
				elif line[0] == 'ypr':
					self.imu.angular_velocity.x = float(line[1])
					self.imu.angular_velocity.y = float(line[2])
					self.imu.angular_velocity.z = float(line[3])
				elif line[0] == 'areal':
					self.imu.linear_acceleration.x = float(line[1])
					self.imu.linear_acceleration.y = float(line[2])
					self.imu.linear_acceleration.z = float(line[3])
		else:
			line = line.split('\t')
			if line[0] == 'ir':
				cmd = self.commands.get(int(line[1]), None)
				if cmd:
					rospy.loginfo('COMMAND=%s' % cmd['code'])
					cmd['proc']()
			elif line[0] == 'vcc':
#				print line[1]
				pwr = arduino_power()
				pwr.arduino_id = ARDUINO_ID
				pwr.vcc = int(line[1])
				rospy.loginfo('VCC=%sV' % (pwr.vcc / 1000.))
				self.pwrpub.publish(pwr)
#		print line
#		self.sendLine('Echo: ' + line)
#		self.transport.write('>>> ')

	def do_forward(self):
		'Forward (0-255)'
		self.cmdpub.publish(drive(ord('f'), 255, 1))

	def do_back(self):
		'Back (0-255)'
		self.cmdpub.publish(drive(ord('b'), 255, 1))

	def do_stop(self):
		'Stop motors'
		self.cmdpub.publish(drive(ord('s'), 0, 1))

	def do_left(self):
		'Step left'
		self.cmdpub.publish(drive(ord('l'), 255, 1))

	def do_right(self):
		'Step right'
		self.cmdpub.publish(drive(ord('r'), 255, 1))

	def do_repeat(self):
		pass

def sayBye():
	print "bye bye."

SerialPort(Processor(), DEVICE, reactor, baudrate=BAUD)

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
