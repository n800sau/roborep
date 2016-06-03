#!/usr/bin/env python

import os, sys, subprocess, json, datetime, time
import redis

text = None

def dbprint(text):
    print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

r = redis.Redis()

state,last_state = r.mget('gate', 'last_gate')

if state:

	try:
		last_state = json.loads(last_state)
	except:
		last_state = {}
	state = json.loads(state)
	r.set('last_gate', json.dumps(state))
	if state['ts'] > last_state.get('ts', 0):
		if last_state.get('label', None) != state['label']:
			if state['label'] in ('open',):
				text = 'gate is ' + state['label']

	if text:
		#cmdlst = ['espeak', '-s', '150', text, '--stdout']
		#cmd = '%s | sox -t wav - -r 44100 -t wav - | aplay -v' % subprocess.list2cmdline(cmdlst)
		cmdlst = ['echo', text]
		cmd = '%s | text2wave -f 44100 |aplay -v' % subprocess.list2cmdline(cmdlst)
#		dbprint(cmd)
		dbprint(text)
#		p = subprocess.Popen(cmd, shell=True)
#		p.wait()
