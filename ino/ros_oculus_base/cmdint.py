#!/usr/bin/env python

import cmd, sys, os, atexit, json, readline, time, serial

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from forklift import run_process, terminate_all
import libcommon_py as common
from conf_ino import configure

class SerReader:

	def __init__(self):
		cfg = configure().as_dict('serial')
		self.s = serial.Serial(cfg['serial_port'], int(cfg.get('baud_rate', 9600)), timeout=5, interCharTimeout=1)
		self.reply_mode = False
		self.replylist = []
		self.final_replylist = []

	def read_replylist(self):
		rs = self.final_replylist
		self.final_replylist = []
		return rs

	def readline(self):
		return self.s.readline()

	def write(self, line):
		self.s.write(line)

	def idle(self):
		if self.s.inWaiting():
			line = self.readline()
			print line

class OculusShell(cmd.Cmd):
	intro = 'Welcome to the oculus shell.	Type help or ? to list commands.\n'
	prompt = '(oculus) '

	def __init__(self):
		cmd.Cmd.__init__(self)
		self.file = None
		self.logfile = None
		cfg = configure().as_dict('serial')
		self.s = serial.Serial(cfg['serial_port'], int(cfg.get('baud_rate', 9600)), timeout=5, interCharTimeout=1)
		self.reply_mode = False
		self.replylist = []
		self.subprocess = run_process(SerReader(), robject=self)

	def write_log(self, line):
		if not self.logfile:
			self.logfile = file('oculus_base.log', 'a+')
		self.logfile.write('%s\n' % line.rstrip())
		self.logfile.flush()

	def on_reply(self, *args):
		print '\n%s' % args

	# ----- oculus commands -----

	def publish(self, cmd, power, secs):
		self.subprocess.write('%s\n' % json.dumps({'command': cmd, 'power': power, 'secs': secs}))

	def do_step_fw(self, arg):
		'Step forward'
		if not arg:
			arg = 0
		self.publish('f', 255, 1)
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_step_back(self, arg):
		'Step back'
		if not arg:
			arg = 0
		self.publish('b', 255, 1)
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_forward(self, arg):
		'Forward (0-255)'
		if not arg:
			arg = 0
		self.publish('f', int(arg), 1)

	def do_back(self, arg):
		'Back (0-255)'
		if not arg:
			arg = 0
		self.publish('b', int(arg), 1)

	def do_step_left(self, arg):
		'Step left'
		if not arg:
			arg = 0
		self.publish('l', 255, 1)
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_step_right(self, arg):
		'Step right'
		if not arg:
			arg = 0
		self.publish('r', 255, 1)
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_setcam(self, arg):
		'Set cam angle [55-80]'
		if not arg:
			arg = 0
		pos = int(arg)
		pos = 80 - pos * (80-55) / 100.
		self.publish('v', pos, 1)

	def do_release_cam(self, arg):
		'Release cam'
		self.publish('w', 0, 1)

	def do_stop(self, arg):
		'Stop motors'
		self.publish('s', 0, 1)

	def do_eof(self, arg):
		print
		self.do_quit(arg)


	# ----- oculus commands end-----

	def do_quit(self, arg):
		'Exit the shell:	 QUIT'
		self.close()
		sys.exit(0)
		return True

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

	def emptyline(self):
		pass

	def close(self):
		if self.file:
			self.file.close()
			self.file = None
		if self.logfile:
			self.logfile.close()
			self.logfile = None

if __name__ == '__main__':
	histfile = os.path.join(os.environ["HOME"], ".oculushist")
	try:
		readline.read_history_file(histfile)
	except IOError:
		pass
	atexit.register(readline.write_history_file, histfile)
	atexit.register(terminate_all)
	del histfile
	OculusShell().cmdloop()
