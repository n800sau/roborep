#!/usr/bin/env python

import socket, json, time

MCAST_GRP = '239.0.0.57'
MCAST_PORT = 12345

OWN_WEATHER = 0xfd4ab5
OUTER_WEATHER = 0xfd0af5
VOL_UP = 0xfdb04f
VOL_DOWN = 0xfd8877

#IRCODE = OWN_WEATHER
IRCODE = VOL_DOWN
#IRCODE = VOL_UP

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
sock.sendto(json.dumps({
	'encoding': 'NEC',
	'ircode': IRCODE,
}), (MCAST_GRP, MCAST_PORT))
print 'sent'
time.sleep(1)
