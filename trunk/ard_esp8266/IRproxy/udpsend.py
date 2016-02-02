#!/usr/bin/env python

import socket

MCAST_GRP = '239.0.0.57'
MCAST_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
sock.sendto("multi data", (MCAST_GRP, MCAST_PORT))
