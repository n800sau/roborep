#!/usr/bin/env python

import cmd, sys, os, serial, atexit, readline, socket, time

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from forklift import run_process, terminate_all
import libcommon_py as common

SERIAL = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A4015M29-if00-port0'
BAUD = 57600

class SerReader:

	def __init__(self):
		self.s = serial.Serial(SERIAL, BAUD, timeout=5, interCharTimeout=1)
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
#			print >>sys.stderr, '%s, %s, %s' % (line, common.REPLY_START_MARKER, line == common.REPLY_START_MARKER)
#			print >>sys.stderr, self.final_replylist
			if self.reply_mode:
				if line.strip() == common.END_MARKER:
					self.final_replylist += self.replylist
					self.replylist = []
					self.reply_mode = False
				else:
					#print >>sys.stderr, line
					self.replylist.append(line)
			else:
				if line.startswith(common.REPLY_MARKER):
					#print >>sys.stderr, line
					self.final_replylist.append(line[len(common.REPLY_MARKER):])
				elif line.strip() == common.REPLY_START_MARKER:
					self.reply_mode = True
				elif line.startswith(common.SERVO_STATE_MARKER):
					self.final_replylist.append(line[len(common.SERVO_STATE_MARKER):])

class OculusShell(cmd.Cmd):
	intro = 'Welcome to the oculus shell.	Type help or ? to list commands.\n'
	prompt = '(oculus) '

	def __init__(self):
		cmd.Cmd.__init__(self)
		self.file = None
		self.logfile = None
		self.subprocess = run_process(SerReader(), robject=self)

	def write_log(self, line):
		if not self.logfile:
			self.logfile = file('oculus_base.log', 'a+')
		self.logfile.write('%s\n' % line.rstrip())
		self.logfile.flush()

	def batstat(self):
		if os.path.exists('/sys/class/power_supply/BAT0'):
			print 'Battery status: %d%%' % (float(file('/sys/class/power_supply/BAT0/charge_now').read())/float(file('/sys/class/power_supply/BAT0/charge_full').read())*100),
			current = int(file('/sys/class/power_supply/BAT0/current_now').read())
			if current == 0:
				print ',charging...'
			else:
				print ',current=%s' % current

	def readreply(self):
		self.batstat()
		i = 0
		while True:
			sys.stdout.write("\x0d%s" % ('*' * i))
			sys.stdout.flush()
			rs = self.subprocess.read_replylist(wait=True)
			if rs:
				break
			time.sleep(1)
			i += 1
			if i > 10:
				break
		print "\x0d%s\n" % (' ' * i)
		return ''.join(rs)

	def print_reply(self):
		reply = self.readreply()
		if reply:
			print 'REPLY:', reply
		else:
			print 'NO REPLY'

	# ----- oculus commands -----

	def do_step_fw(self, arg):
		'Step forward'
		self.subprocess.write('f%c\n' % 255)
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_step_back(self, arg):
		'Step back'
		self.subprocess.write('b%c\n' % 255)
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_forward(self, arg):
		'Forward (0-255)'
		self.subprocess.write('f%c\n' % int(arg))

	def do_back(self, arg):
		'Back (0-255)'
		self.subprocess.write('b%c\n' % int(arg))

	def do_step_left(self, arg):
		'Step left'
		self.subprocess.write('l%c\n' % 255)
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_step_right(self, arg):
		'Step right'
		self.subprocess.write('r%c\n' % 255)
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_setcam(self, arg):
		'Set cam angle [75-100]'
		self.subprocess.write('v%c\n' % int(arg))
		self.print_reply()

	def do_release_cam(self, arg):
		'Release cam'
		self.subprocess.write('w\n')
		self.print_reply()

	def do_version(self, arg):
		'Get firmware version'
		self.subprocess.write('y\n')
		self.print_reply()

	def do_stop(self, arg):
		'Stop motors'
		self.subprocess.write('s\n')
		self.print_reply()

	def do_echo(self, arg):
		'Echo on/off: ECHO <ON|OFF>'
		if arg.lower() == 'on':
			self.subprocess.write('e1\n')
		elif arg.lower() == 'off':
			self.subprocess.write('e0\n')
		self.print_reply()


	# ----- oculus commands end-----

	def do_quit(self, arg):
		'Exit the shell:	 QUIT'
		self.close()
		sys.exit(0)
		return True

	def do_eof(self, arg):
		print
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

	def emptyline(self):
		self.batstat()

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
