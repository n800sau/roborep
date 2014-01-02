#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from time import time
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from oculus2wd.msg import battery

VOICE = 'voice_kal_diphone'
#VOICE = 'voice_rab_diphone' #too quiet
VOICE_RU = 'voice_msu_ru_nsh_clunits' #needs festvox-ru to be installed

last_say_time = 0

def say(words, voice=None):
	print words, voice or VOICE
	global last_say_time
	soundhandle.say(words, voice or VOICE)
	last_say_time = time()

def callback(msg):
#		rospy.loginfo(rospy.get_name() + ": Received %s" % msg.status)
	global last_say_time
	tdiff = time() - last_say_time
	if msg.level < 20:
		if tdiff > 10:
			say('Спасите! Верните меня на базу!', VOICE_RU)
	elif msg.level < 40:
		if tdiff > 2:
			say('S.O.S.! I need power')
	elif msg.level < 50:
		if tdiff > 10:
			say('%d percent of power. I\'m hungry.' % msg.level)

# Ordered this way to minimize wait time.
rospy.init_node('power_help', anonymous = True)
soundhandle = SoundClient()

rospy.Subscriber("/oculus2wd/battery", battery, callback)
rospy.spin()





