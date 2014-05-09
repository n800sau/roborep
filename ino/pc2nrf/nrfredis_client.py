#!/usr/bin/env python

import sys, os, redis, traceback, time, csv
from subprocess import check_call

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import libcommon_py as common

MAX2READ = 100

if __name__ == '__main__':

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
			for marker in r.smembers('s.queues'):
				lasttimestamp = None
				if marker not in queues:
					queues[marker] = {'lastel' : None}
					queues[marker]['file'] = file(os.path.join(os.path.dirname(__file__), marker + '.csv'), 'a+')
					queues[marker]['csv'] = csv.writer(queues[marker]['file'])
					# read last csv file line
					for row in csv.reader(queues[marker]['file']):
						ldict = dict(zip(row[::2], row[1::2]))
						lasttimestamp = int(ldict['secs'])
				qdata = queues[marker]
				els = r.lrange(marker, 0, MAX2READ)
				if qdata['lastel']:
					# remove old elements
					try:
						lastndx = els.index(qdata['lastel'])
					except ValueError:
						lastndx = -1
					if lastndx >= 0:
						els = els[:lastndx]
				if els:
					# save the last element for the future reference
					qdata['lastel'] = els[0]
				# write remains to csv
				for line in reversed(els):
					line = [s.strip() for s in line.split(common.DATA_SEPARATOR)[1:]]
					ldict = dict(zip(line[::2], line[1::2]))
					if lasttimestamp is None or lasttimestamp < int(ldict['secs']):
#						print time.strftime('%d/%m/%Y %H:%M:%S', time.localtime(float(ldict['secs'])))
						qdata['csv'].writerow(line + [
								time.strftime('%d/%m/%Y %H:%M:%S', time.localtime(float(ldict['secs']))),
								time.strftime('%d/%m/%Y %H:%M:%S')
							]
						)
#					else:
#						print 'Old data', time.strftime('%d/%m/%Y %H:%M:%S', time.localtime(float(ldict['secs'])))
					qdata['file'].flush()
		except KeyboardInterrupt:
			break
		except:
			traceback.print_exc()

	for qdata in queues.values():
		qdata['file'].close()

