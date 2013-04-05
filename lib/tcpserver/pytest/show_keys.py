#!/usr/bin/env python

import socket, json, time, pprint

s = socket.create_connection(('115.70.59.149', 7980))
s.sendall(json.dumps({'cmd': 'send_full_data'}))
f = s.makefile()
for line in f:
	dobj = json.loads(line)
	print dobj['s_timestamp']
	for k,v in dobj.items():
		print k
#		if isinstance(v, dict):
#			for vk,vv in v.items():
#				print "\t", vk
#				if isinstance(vv, dict):
#					for vvk in vv.keys():
#						print "\t\t", vvk


