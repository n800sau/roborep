#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os
import rospy
from time import time, sleep
from std_msgs.msg import Float32
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

VOICE = 'voice_kal_diphone'
#VOICE = 'voice_rab_diphone' #too quiet
VOICE_RU = 'voice_msu_ru_nsh_clunits' #needs festvox-ru to be installed

#to keep pauses between sayings
last_say_time = 0
english_mode = True

def say(words, voice=None):
#	print words
	global last_say_time, english_mode
	last_say_time = time()
	english_mode = not english_mode
	soundhandle.say(words, voice or VOICE)


def callback(data):
	global last_say_time, english_mode
	tdiff = time() - last_say_time
	if data.data < 0.1 and tdiff > 20:
		# darkness
		if english_mode:
			say('Turn on the light, please.')
		else:
			say('Включите свет, пожалуйста.', VOICE_RU)

# Ordered this way to minimize wait time.
rospy.init_node('light_help', anonymous = True)
soundhandle = SoundClient()

rospy.Subscriber('/brightness', Float32, callback)

rospy.spin()
