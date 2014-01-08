#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy
from time import time, sleep
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from oculus2wd.msg import battery

LEVEL_OFFSET = 0 #for debug purpose it is possible to offset levels by this value
LEVELS = (10, 20, 40)
#minimal battery level
LEVEL_MIN = 5

VOICE = 'voice_kal_diphone'
#VOICE = 'voice_rab_diphone' #too quiet
VOICE_RU = 'voice_msu_ru_nsh_clunits' #needs festvox-ru to be installed

TIME2OFF = 120

#to keep pauses between sayings
last_say_time = 0

#to stop repeating the same level
last_level = 0

#to have timeout befor switch off
uptime2off = None

alert_mode = False
english_mode = True
powering_off = False

def say(words, voice=None):
	global last_say_time, english_mode
	last_say_time = time()
	english_mode = not english_mode
	soundhandle.say(words, voice or VOICE)

def callback(msg):
#		rospy.loginfo(rospy.get_name() + ": Received %s" % msg.status)
	global last_say_time, alert_mode, english_mode, last_level, uptime2off, powering_off
	tdiff = time() - last_say_time
	if msg.discharging:
		if msg.level < LEVELS[0] + LEVEL_OFFSET:
			alert_mode = True
			uptime = float(file('/proc/uptime').read().split()[0])
			if uptime2off is None:
				uptime2off = uptime
			if (tdiff > 10 and not english_mode) or tdiff > 20:
				time2off = TIME2OFF + uptime2off - uptime
				# let it some time (5 minutes) to pass after turning on to let the user to cancel the node
				if time2off < 0 or msg.level < LEVEL_MIN + LEVEL_OFFSET:
					powering_off = True
					say('Прощай, жестокий мир. Я выключаюсь!', VOICE_RU)
					sleep(15)
					say('Good bye, cruel world. Powering off.')
					sleep(10)
					os.system('sudo poweroff')
				else:
					if english_mode:
						say('%d seconds remaining' % time2off)
					else:
						say('До выключения осталось %d секунд.' % time2off, VOICE_RU)
		elif msg.level < LEVELS[1] + LEVEL_OFFSET:
			alert_mode = True
			if (tdiff > 10 and not english_mode) or tdiff > 20:
				if english_mode:
					say('S.O.S.! I need power. Return me to base.')
				else:
					say('Спасите! Верните меня на базу!', VOICE_RU)
		elif msg.level < LEVELS[2] + LEVEL_OFFSET:
			alert_mode = True
			if last_level != msg.level:
				say('%d percent of power.' % msg.level)
		last_level = msg.level
	else:
		if alert_mode:
			if powering_off:
				say('Too late. I has sad.')
				sleep(10)
			else:
				say('Ok! I\'m happy now.')
			alert_mode = False
		uptime2off = None
		last_level = 0

# Ordered this way to minimize wait time.
rospy.init_node('power_help', anonymous = True)
soundhandle = SoundClient()

rospy.Subscriber("/oculus2wd/battery", battery, callback)
rospy.spin()
