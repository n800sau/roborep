#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from time import time
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from oculus2wd.msg import battery

LEVEL_OFFSET = 50
LEVELS = (20, 40)

VOICE = 'voice_kal_diphone'
#VOICE = 'voice_rab_diphone' #too quiet
VOICE_RU = 'voice_msu_ru_nsh_clunits' #needs festvox-ru to be installed

last_say_time = 0
last_level = 0

alert_mode = False
english_mode = True

def say(words, voice=None):
	global last_say_time, english_mode
	soundhandle.say(words, voice or VOICE)
	last_say_time = time()
	english_mode = not english_mode

def callback(msg):
#		rospy.loginfo(rospy.get_name() + ": Received %s" % msg.status)
	global last_say_time, alert_mode, english_mode, last_level
	tdiff = time() - last_say_time
	if msg.discharging:
		if msg.level < LEVELS[0] + LEVEL_OFFSET:
			alert_mode = True
			if tdiff > 10:
				if english_mode:
					say('S.O.S.! I need power. Return me to base.')
				else:
					say('Спасите! Верните меня на базу!', VOICE_RU)
		elif msg.level < LEVELS[1] + LEVEL_OFFSET:
			alert_mode = True
			if last_level != msg.level:
				say('%d percent of power.' % msg.level)
		last_level = msg.level
	else:
		if alert_mode:
			say('Ok! I\'m happy now.')
			alert_mode = False
		last_level = 0

# Ordered this way to minimize wait time.
rospy.init_node('power_help', anonymous = True)
soundhandle = SoundClient()

rospy.Subscriber("/oculus2wd/battery", battery, callback)
rospy.spin()





