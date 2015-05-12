#!/usr/bin/env python

import redis
import json
import csv
import pycmds
import time
import datetime

r = redis.Redis()

of = file('vecs.csv', 'w')
ow = csv.writer(of)

for d in reversed(r.lrange('6050', 0, -1)):
	jd = json.loads(d)
	if jd['cmd'] == pycmds.R_ACC_3F:
		row = [
			datetime.datetime.fromtimestamp(jd['t']).strftime('%d/%m/%Y %H:%M:%S.%f'),
			jd['t'],
			jd['millis'],
			jd['vals'][0],
			jd['vals'][1],
			jd['vals'][2],
		]
		ow.writerow(row)

of.close()
