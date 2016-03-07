#!/usr/bin/env python

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
IRCODE_LIST = (OWN_WEATHER, OUTER_WEATHER)

def get_last_data():
	r = redis.Redis('192.168.2.80')
	data = r.hgetall('dht11_garage')
	last_time = max([int(k) for k in data.keys()])
	rs = json.loads(data[str(last_time)])
	rs['timestamp'] = last_time
	return rs

MCAST_GRP = '239.0.0.57'
MCAST_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))  # use MCAST_GRP instead of '' to listen only
                             # to MCAST_GRP, not all groups on MCAST_PORT
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

last_time = None
p = None
while True:
	try:
		print 'Waiting...'
		data = sock.recv(1000)
		data = json.loads(data)
		print 'encoding:%s, value:0x%4.4x' % (data['encoding'], data['ircode'])
		if data['encoding'] == MODEL and data['ircode'] in IRCODE_LIST:
			if last_time is None or last_time + 2 < time.time():
				print 'Pass'
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
					p = subprocess.Popen(['espeak', '-s', '150', text], stdout=nullf, stderr=nullf)
			else:
				print 'Too soon'
		else:
			print 'Wrong code'
	except Exception, e:
		if isinstance(e, KeyboardInterrupt):
			raise
		traceback.print_exc()

