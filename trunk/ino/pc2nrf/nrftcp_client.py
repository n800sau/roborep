#!/usr/bin/env python

import sys, os, socket, traceback, time, csv
from subprocess import check_call

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import libcommon_py as common

if __name__ == '__main__':

	files = {}
	cfg = configure(os.path.dirname(__file__)).as_dict('serial2tcp')
	tcp_host = cfg.get('tcp_host', 'localhost')
	tcp_port = cfg['tcp_port']

	s = None
	for res in socket.getaddrinfo(tcp_host, tcp_port, socket.AF_UNSPEC, socket.SOCK_STREAM):
		af, socktype, proto, canonname, sa = res
		try:
			s = socket.socket(af, socktype, proto)
		except socket.error as msg:
			s = None
			continue
		try:
			s.connect(sa)
		except socket.error as msg:
			s.close()
			s = None
			continue
		break
	if s is None:
		print 'could not open socket'
		sys.exit(1)
	try:
		sfile = s.makefile()
		while True:
			line = sfile.readline().strip()
			if line:
				marker = line[0]
				try:
					if marker in (common.CONTROLLER_STATE_MARKER, common.REPLY_MARKER):
						line = line.split(common.DATA_SEPARATOR)[1:]
						ldict = dict(zip(line[::2], line[1::2]))
						print time.strftime('%d/%m/%Y %H:%M:%S', time.localtime(float(ldict['secs'])))
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
	finally:
		s.close()
