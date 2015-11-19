#!/usr/bin/env python

import json, time
from lib.sensor_const import *
import redis

r = redis.Redis()


for squeue in (
		ACCEL_REDIS_QUEUE,
		COMPASS_REDIS_QUEUE,
		GYRO_REDIS_QUEUE):

	jrange = r.lrange(squeue, 0, -1)
	j1 = json.loads(jrange[0])
	j2 = json.loads(jrange[-2])
	j3 = json.loads(jrange[-1])

	print '%s %.2f times per sec (size:%d, recent:%s, last_diff=%.3f)' % (squeue,
			len(jrange) / (j3['time']-j1['time']),
			len(jrange),
			time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(j3['time'])),
			j3['time']-j2['time']
		)
