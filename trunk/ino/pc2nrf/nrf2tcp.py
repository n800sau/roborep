#!/usr/bin/env python

import sys, os, serial, socket, traceback, time
from subprocess import check_call

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import libcommon_py as common

if __name__ == '__main__':

	cfg = configure().as_dict('serial')
	s_port = cfg['serial_port']
	s_rate = int(cfg.get('baud_rate', 9600))

	cfg = configure().as_dict('serial2tcp')
	tcp_host = cfg.get('tcp_host', '0.0.0.0')
	tcp_port = cfg['tcp_port']

	print s_port, s_rate
#	check_call(['stty', '-F', s_port, str(s_rate)])
	serdev = serial.Serial(s_port, s_rate, timeout=5, interCharTimeout=1)
	while True:
		line = serdev.readline()
		if line.strip() == common.READY_MARKER:
			break
	settime_cmd = '%s%d\r\nsetTime %d\r\n%s\r\n' % (common.ADDRESS_MARKER, common.PC2NRF_NODE, time.time() * 1000, common.END_MARKER)
	print serdev.write(settime_cmd), 'bytes written'
	serdev.flushOutput()
#	while True:
#		print 'waiting...'
#		print serdev.readline()
#	sys.exit()
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


