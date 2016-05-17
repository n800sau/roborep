#!/usr/bin/env python

import socket, json, time

MCAST_GRP = '239.0.0.57'
MCAST_PORT = 12345

from nec_codes import *

#IRCODE = WATER_SPIT
IRCODE = OWN_WEATHER
#IRCODE = VOL_DOWN
#IRCODE = VOL_UP

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
sock.sendto(json.dumps({
	'encoding': MODEL,
	'ircode': IRCODE,
}), (MCAST_GRP, MCAST_PORT))
print 'sent'
time.sleep(1)
