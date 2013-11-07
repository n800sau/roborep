#!/usr/bin/env python

import cmd, sys, os, serial, io, atexit, readline

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))

import libcommon_py as common

SERIAL = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1014RKM-if00-port0'
BAUD = 57600

class StickShell(cmd.Cmd):
	intro = 'Welcome to the stick shell.	Type help or ? to list commands.\n'
	prompt = '(stick) '

	def __init__(self):
		cmd.Cmd.__init__(self)
		self.s = serial.Serial(SERIAL, BAUD, timeout=1, interCharTimeout=1)
		self.sio = io.TextIOWrapper(io.BufferedRWPair(self.s, self.s))
		self.file = None

	def readreply(self):
		rs = []
		reply_mode = False
		while True:
			reply = self.s.readline()
			if not reply:
				rs.append('*** timeout ***')
				break
			if reply_mode:
				if reply == common.REPLY_END_MARKER:
					reply_mode = False
					break
				else:
					rs.append(reply)
			else:
				if reply.startswith(common.REPLY_MARKER):
					rs.append(reply[len(common.REPLY_MARKER):])
					break
				elif reply == common.REPLY_START_MARKER:
					reply_mode = True
		return '\n'.join(rs)

	# ----- basic stick commands -----

	def do_low_pos(self, arg):
		'Set/get low position (0-180):	 LOW_POS 30 or LOW_POS'
		if len(arg) > 0:
			self.s.write('%slow_set %d' % (common.CMD_MARKER, int(arg)))
		else:
			self.s.write('%slow_get' % common.CMD_MARKER)
		reply = self.readreply()
		print 'REPLY:', reply

	def do_reset(self, arg):
		'Reset positions to 90s:	RESET'
		self.s.write('%sreset' % common.CMD_MARKER)
		reply = self.readreply()
		print 'REPLY:', reply

	def do_quit(self, arg):
		'Exit the shell:	 QUIT'
		self.close()
		sys.exit(0)
		return True

	def do_eof(self, arg):
		self.do_quit(arg)

	# ----- record and playback -----

	def do_record(self, arg):
		'Save future commands to filename:	RECORD destroy.cmd'
		self.file = open(arg, 'w')

	def do_playback(self, arg):
		'Playback commands from a file:	 PLAYBACK destroy.cmd'
		self.close()
		with open(arg) as f:
			self.cmdqueue.extend(f.read().splitlines())

	def precmd(self, line):
		line = line.lower()
		if self.file and 'playback' not in line:
			print >>self.file, line
		return line

	def close(self):
		if self.file:
			self.file.close()
			self.file = None

def parse(arg):
	'Convert a series of zero or more numbers to an argument tuple'
	return tuple(map(int, arg.split()))

if __name__ == '__main__':
	histfile = os.path.join(os.environ["HOME"], ".stickhist")
	try:
		readline.read_history_file(histfile)
	except IOError:
		pass
	atexit.register(readline.write_history_file, histfile)
	del histfile
	StickShell().cmdloop()
