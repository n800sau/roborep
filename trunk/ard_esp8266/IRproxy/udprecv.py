#!/usr/bin/env python

import sys
import datetime
import socket
import struct
import json
import subprocess
import time
import redis
import traceback
from get_weather import get_weather_list

MODEL = 'NEC'
OWN_WEATHER = 0xfd4ab5
OUTER_WEATHER = 0xfd0af5
VOL_UP = 0xfdb04f
VOL_DOWN = 0xfd8877
IRCODE_LIST = (OWN_WEATHER, OUTER_WEATHER, VOL_UP, VOL_DOWN)

MCAST_GRP = '239.0.0.57'
MCAST_PORT = 12345

def dbprint(text):
    print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

def get_last_data():
	r = redis.Redis('192.168.2.80')
	data = r.hgetall('dht11_garage')
	last_time = max([int(k) for k in data.keys()])
	rs = json.loads(data[str(last_time)])
	rs['timestamp'] = last_time
	return rs

def make_sock():

	i = 1
	while True:

		try:
			sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
			sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
			sock.bind(('', MCAST_PORT))  # use MCAST_GRP instead of '' to listen only
                             # to MCAST_GRP, not all groups on MCAST_PORT
			mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

			sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
			break

		except:
			traceback.print_exc()
			time.sleep(5)
			dbprint('Attempt %d to bind' % i)
			i += 1

	return sock

if __name__ == '__main__':

	sock = make_sock()

	last_time = None
	p = None
	while True:
		try:
			dbprint('Waiting...')
			data = sock.recv(1000)
			data = json.loads(data)
			dbprint('encoding:%s, value:0x%4.4x' % (data['encoding'], data['ircode']))
			if data['encoding'] == MODEL and data['ircode'] in IRCODE_LIST:
				subprocess.call(['beep', '-f', '555', '-l', '460'])
				if last_time is None or last_time + 2 < time.time():
					dbprint('Pass')
					if data['ircode'] == VOL_UP:
						subprocess.call(["amixer", "-D", "pulse", "sset", "Master", "5%+"])
					elif data['ircode'] == VOL_DOWN:
						subprocess.call(["amixer", "-D", "pulse", "sset", "Master", "5%-"])
					else:
						if not p is None:
							p.wait()
							p = None
						last_time = time.time()
						nullf = file('/dev/null', 'w')
						text = None
						if data['ircode'] == OWN_WEATHER:
							last_data = get_last_data()
							timestamp = time.localtime(last_data['timestamp'])
							text = 'At %d hour, %d minutes, the temperature outside is %d degrees, humidity is %d' % (
								timestamp.tm_hour, timestamp.tm_min, last_data['t'], last_data['h'])
						elif data['ircode'] == OUTER_WEATHER:
							text = '.\n'.join(get_weather_list('Kirrawee'))
						if text:
							#cmdlst = ['espeak', '-s', '150', text, '--stdout']
							#cmd = '%s | sox -t wav - -r 44100 -t wav - | aplay -v' % subprocess.list2cmdline(cmdlst)
							cmdlst = ['echo', text]
							cmd = '%s | text2wave -f 44100 |aplay -v' % subprocess.list2cmdline(cmdlst)
							dbprint(cmd)
							p = subprocess.Popen(cmd, shell=True)
				else:
					dbprint('Too soon')
			else:
				dbprint('Wrong code')
		except Exception, e:
			if isinstance(e, KeyboardInterrupt):
				raise
			traceback.print_exc()

