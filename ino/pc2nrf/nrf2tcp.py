#!/usr/bin/env python

import sys, os, serial, socket, traceback, time

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
from subprocess import check_call

if __name__ == '__main__':

	cfg = configure().as_dict('serial')
	s_port = cfg['serial_port']
	s_rate = int(cfg.get('baud_rate', 9600))

	cfg = configure().as_dict('serial2tcp')
	tcp_host = cfg.get('tcp_host', '0.0.0.0')
	tcp_port = cfg['tcp_port']

	check_call(['stty', '-F', s_port, str(s_rate)])
	serdev = serial.Serial(s_port, s_rate, timeout=5, interCharTimeout=1)
#	cmdlist = ['nc.traditional', '-l', '-p', tcp_port, tcp_host]
	cmdlist = ['nc', '-C', '-k', '-l', tcp_port]
	while True:
		print 'Starting: %s' % ' '.join(cmdlist)
		print 'To connect run: nc %s %s' % (socket.gethostname(), tcp_port)
		try:
			check_call(cmdlist, stdin = serdev, stdout=serdev)
		except KeyboardInterrupt:
			break
		except:
			traceback.print_exc()
			time.sleep(1)


