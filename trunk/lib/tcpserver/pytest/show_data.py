#!/usr/bin/env python

import socket, json, time, pprint

s = socket.create_connection(('115.70.59.149', 7980))
s.sendall(json.dumps({'cmd': 'send_full_data', 'interval': 100, 'count': 100}))
f = s.makefile()
for line in f:
	dobj = json.loads(line)
#	print dobj['s_timestamp']
#	print dobj.keys()
	print pprint.pformat(dobj)


