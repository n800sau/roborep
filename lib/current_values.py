#!/usr/bin/env python

import time, redis, json

r = redis.Redis()

snames = [
	'hmc5883l',
	'bmp085',
	'adxl345',
]

def get_last(sname, no_older_than=None):
	rs = r.lrange('%s.js.obj' % sname, 0, 0)
	if rs:
		rs = json.loads(rs[0])
		if not no_older_than is None:
#			print float(rs['timestamp']) + no_older_than, time.time()
			if float(rs['timestamp']) + no_older_than < time.time():
				rs = None
	return rs

for s in snames:
	ts = r.get(s + '.timestamp')
	print s, ts
	vs = get_last(s, 3)
	print vs
#	print time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(vs['timestamp'])), time.strftime('%Y-%m-%d %H:%M:%S'), r.get('hmc5883l.timestamp')
#	print ts
#	print time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(int(vs['timestamp'])))
#	print time.strftime('%Y-%m-%d %H:%M:%S')
#	print vs['timestamp']
#	print time.time()
#else:
#	print 'too old'
