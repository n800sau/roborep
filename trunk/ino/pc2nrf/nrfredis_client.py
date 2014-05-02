#!/usr/bin/env python

import sys, os, redis, traceback, time, csv
from subprocess import check_call

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import libcommon_py as common

MAX2READ = 100

if __name__ == '__main__':

	files = {}

	cfgobj = configure(os.path.dirname(__file__))
	cfg = cfgobj.as_dict('serial2redis')
	params = {'host': cfg.get('redis_host', 'localhost')}
	redis_port = cfg.get('redis_port', None)
	if redis_port:
		params['port'] = redis_port

	r = redis.Redis(**params)

	queues = {}
	while True:
		try:
			for rk in r.smembers('s.queues'):
				els = r.lrange(rk, 0, MAX2READ)
				lastel = (queues.get(rk, []) + [None])[0]
				try:
					lastndx = els.index(lastel)
				except ValueError:
					lastndx = -1
				if lastndx >= 0:
					#remove old elements
					els = els[:lastndx]
				if els:
					queues[rk] = els
				#process new elements
				for line in reversed(els):
					if line:
						marker = line[0]
						line = line.split(common.DATA_SEPARATOR)[1:]
						ldict = dict(zip(line[::2], line[1::2]))
#						print time.strftime('%d/%m/%Y %H:%M:%S', time.localtime(float(ldict['secs'])))
						if marker not in files:
							files[marker] = {}
							files[marker]['file'] = file(os.path.join(os.path.dirname(__file__), marker + '.csv'), 'a')
							files[marker]['csv'] = csv.writer(files[marker]['file'])
						files[marker]['csv'].writerow([
								time.strftime('%d.%m.%Y %H:%M:%S'),
								time.strftime('%d/%m/%Y %H:%M:%S', time.localtime(float(ldict['secs'])))
							] + [s.strip() for s in line])
						files[marker]['file'].flush()
		except KeyboardInterrupt:
			break
		except:
			traceback.print_exc()
