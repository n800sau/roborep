#!/usr/bin/env python

import socket, json, time, pprint

host = '115.70.59.149'
port = 7980

#host = 'beaglebone'
#port = 7880

keylist = [
#'l3g4200d.js.obj',
#'hmc5883l.js.obj',
'bmp085.js.obj',
#'lsm303.js.obj',
#'adxl345.js.obj',
#'mpu6050.js.obj',
#'kalman.js.obj',
#'mag3110.js.obj',
]

s = socket.create_connection((host, port))
s.sendall(json.dumps({'cmd': 'send_full_data', 'interval': 100, 'count': 100}))
f = s.makefile()
for line in f:
	dobj = json.loads(line)
	print dobj['s_timestamp']
	for k in keylist:
		if k in dobj:
			v = dobj[k]
			if isinstance(v, dict):
				print k
				for vk,vv in v.items():
					if isinstance(vv, dict):
						print "\t", vk
						for vvk,vvv in vv.items():
							print "\t\t%s = %s" % (vvk, vvv)
					else:
						print "\t%s = %s" % (vk, vv)
			else:
				print "%s = %s" % (k, v)


