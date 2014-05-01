#!/usr/bin/env python

import sys, os, serial, socket, traceback, time
from subprocess import check_call

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from conf_ino import configure
import libcommon_py as common

if __name__ == '__main__':

	cfgobj = configure(os.path.dirname(__file__))

	cfg = cfgobj.as_dict('serial')
	s_port = cfg['serial_port']
	s_rate = int(cfg.get('baud_rate', 9600))

	cfg = cfgobj.as_dict('serial2tcp')
	tcp_host = cfg.get('tcp_host', '0.0.0.0')
	tcp_port = cfg['tcp_port']

#	print s_port, s_rate
#	check_call(['stty', '-F', s_port, str(s_rate)])
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
				print repl
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


