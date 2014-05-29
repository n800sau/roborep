#!/usr/bin/env python

import sys, os, serial, redis, traceback, time
from subprocess import check_call

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import libcommon_py as common

MAX_QUEUE_SIZE = 1000
MESSAGE_CHAN = 'nrf'

if __name__ == '__main__':

	cfgobj = configure(os.path.dirname(__file__))

	cfg = cfgobj.as_dict('serial')
	s_port = cfg['serial_port']
	s_rate = int(cfg.get('baud_rate', 9600))

	cfg = cfgobj.as_dict('serial2redis')
	params = {'host': cfg.get('redis_host', 'localhost')}
	redis_port = cfg.get('redis_port', None)
	if redis_port:
		params['port'] = redis_port

	cleaned = []

	r = redis.Redis(**params)

	serdev = serial.Serial(s_port, s_rate, timeout=1, interCharTimeout=1)
	repeat = True
	while repeat:
		settime_cmd = '%s%d\n%ssetTime %d\n%s\n' % (common.ADDRESS_MARKER, common.PC2NRF_NODE, common.DATA_MARKER, time.time(), common.END_MARKER)
#		print settime_cmd
		print serdev.write(settime_cmd), 'bytes written'
		serdev.flushOutput()
		curtime = time.time()
		while curtime + 3 > time.time():
#			print 'waiting...'
			repl = serdev.readline().strip()
			if repl:
				if repl == common.ACK_MARKER:
					repeat = False
					break
#				print repl
	while True:
		try:
			line = serdev.readline().strip()
			if line:
				marker = line[0]
				if marker:
					rk = 'q.' + marker
					if rk not in cleaned:
						r.delete(rk)
						cleaned.append(rk)
					r.lpush(rk, line)
					print line
					r.ltrim(rk, 0, MAX_QUEUE_SIZE - 1)
					r.sadd('s.queues', rk)
					r.publish(MESSAGE_CHAN, rk)
		except KeyboardInterrupt:
			break
		except:
			traceback.print_exc()
			time.sleep(1)


