#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os, signal
import rospy
from time import time, sleep
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Imu
from nanaybot.msg import drive_status, drive, arduino_power

from twisted.internet import reactor, defer
from twisted.protocols import basic
from twisted.internet.task import LoopingCall
from twisted.internet.protocol import ClientFactory

HOST = 'localhost'
PORT = 9700

ARDUINO_ID = 'imu_ir_tcp'


# Ordered this way to minimize wait time.
rospy.init_node('imu_ir', anonymous = True)
soundhandle = SoundClient()

class Processor(basic.LineReceiver):

	def __init__(self, *args, **kwds):
		self.commands = {
			0x40BF906F: 'FORWARD',
			0xFD609F: 'FORWARD',
			0x40BF807F: 'BACK',
			0xFD6897: 'BACK',
			0x40BF50AF: 'LEFT',
			0xFD5AA5: 'LEFT',
			0x40BF10EF: 'RIGHT',
			0xFDD827: 'RIGHT',
			0x40BFD02F: 'OK',
			0xFD58A7: 'OK',
			0x40BF18E7: 'CAM_UP',
			0xFDB04F: 'CAM_UP',
			0x40BFD827: 'CAM_DOWN',
			0xFD8877: 'CAM_DOWN',
			0x40BFB04F: 'CAM_RELEASE',
			0xFDA05F: 'CAM_RELEASE',
			0xFFFFFFFF: 'REPEAT',
		}
		self.actions = {
			'FORWARD': self.do_forward,
			'BACK': self.do_back,
			'LEFT': self.do_left,
			'RIGHT': self.do_right,
			'OK': self.do_stop,
			'CAM_UP': self.do_camup,
			'CAM_DOWN': self.do_camdown,
			'CAM_RELEASE': self.do_release_cam,
		}
		self.pub = rospy.Publisher('/nanaybot/imu_raw', Imu)
		self.cmdpub = rospy.Publisher('/nanaybot/base_command', drive)
		self.pwrpub = rospy.Publisher('/nanaybot/power', arduino_power)
		self.imu = None
		self.last_cmd = None
		self.campos = 50

	def connectionMade(self):
		rospy.loginfo("TCP port connected")
#		self.transport.write('>>> ')

	def lineReceived(self, line):
		line = line.strip()
		if line:
			if line == 'START':
				self.imu = Imu()
				self.imu.header.stamp = rospy.Time.now()
			elif not self.imu is None:
				if line == 'END':
					print 'IMU=', self.imu
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
#				elif line[0] == 'acc':
						self.imu.linear_acceleration.x = float(line[1])
						self.imu.linear_acceleration.y = float(line[2])
						self.imu.linear_acceleration.z = float(line[3])
			else:
				line = line.split('\t')
				if line[0] == 'ir':
					rospy.logdebug('HEX=%X' % int(line[1]))
#				print 'HEX=%X' % int(line[1])
					code = self.commands.get(int(line[1]), None)
					if code:
						rospy.loginfo('COMMAND=%s' % code)
						if code == 'REPEAT':
							cmd = self.last_cmd
						else:
							cmd = self.actions[code]
							self.last_cmd = cmd
						if cmd:
							cmd()
					else:
						self.last_cmd = None
				elif line[0] == 'vcc':
#					print line[1]
					pwr = arduino_power()
					pwr.arduino_id = ARDUINO_ID
					pwr.vcc = int(line[1])
#					rospy.loginfo('VCC=%sV' % (pwr.vcc / 1000.))
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
		self.cmdpub.publish(drive(ord('l'), 150, 1))

	def do_right(self):
		'Step right'
		self.cmdpub.publish(drive(ord('r'), 150, 1))

	def do_camup(self):
		self.campos += 5
		if self.campos > 100:
			self.campos = 100
		self.do_setcam()

	def do_camdown(self):
		self.campos -= 5
		if self.campos < 0:
			self.campos = 0
		self.do_setcam()

	def do_setcam(self):
		'Set cam angle [55-80]'
		pos = 80 - self.campos * (80-55) / 100.
		self.cmdpub.publish(drive(ord('v'), pos, 1))

	def do_release_cam(self):
		'Release cam'
		self.cmdpub.publish(drive(ord('w'), 0, 1))


def sayBye():
	print "bye bye."


f = ClientFactory()
f.protocol = Processor

reactor.connectTCP(HOST, PORT, f)

#SerialPort(Processor(), DEVICE, reactor, baudrate=BAUD)

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
