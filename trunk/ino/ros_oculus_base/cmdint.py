#!/usr/bin/env python

import cmd, sys, os, atexit, readline, socket, time

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from forklift import run_process, terminate_all
import libcommon_py as common

import rospy
from oculus2wd.msg import drive_status
from oculus2wd.msg import drive

class OculusShell(cmd.Cmd):
	intro = 'Welcome to the oculus shell.	Type help or ? to list commands.\n'
	prompt = '(oculus) '

	def __init__(self):
		cmd.Cmd.__init__(self)
		rospy.init_node('drive_node')
		self.pub = rospy.Publisher('/oculus2wd/base_command', drive)
		rospy.Subscriber("/oculus2wd/base_status", drive_status, self.on_reply)
		print dir(drive_status)
		self.file = None
		self.logfile = None

	def write_log(self, line):
		if not self.logfile:
			self.logfile = file('oculus_base.log', 'a+')
		self.logfile.write('%s\n' % line.rstrip())
		self.logfile.flush()

	def batstat(self):
		status = 'Unknown'
		energy_full = None
		energy_now = None
		for line in file('/sys/class/power_supply/BAT0/uevent'):
			line = line.strip()
			if line:
				n,v = line.split('=')
				if n in ('POWER_SUPPLY_STATUS',):
					status = v
				elif n in ('POWER_SUPPLY_CHARGE_FULL', 'POWER_SUPPLY_ENERGY_FULL'):
					energy_full = int(v)
				elif n in ('POWER_SUPPLY_CHARGE_NOW', 'POWER_SUPPLY_ENERGY_NOW'):
					energy_now = int(v)
		if energy_full and energy_now:
			print 'Status %g%%, %s' % (float(energy_now) /energy_full * 100, status)

	def on_reply(self, *args):
		print '\n%s' % args

	# ----- oculus commands -----

	def do_step_fw(self, arg):
		'Step forward'
		self.pub.publish(drive(ord('f'), 255, 1))
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_step_back(self, arg):
		'Step back'
		self.pub.publish(drive(ord('b'), 255, 1))
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_forward(self, arg):
		'Forward (0-255)'
		self.pub.publish(drive(ord('f'), int(arg), 1))

	def do_back(self, arg):
		'Back (0-255)'
		self.pub.publish(drive(ord('b'), int(arg), 1))

	def do_step_left(self, arg):
		'Step left'
		self.pub.publish(drive(ord('l'), 255, 1))
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_step_right(self, arg):
		'Step right'
		self.pub.publish(drive(ord('r'), 255, 1))
		time.sleep(min(1,float(arg)))
		self.do_stop('')

	def do_setcam(self, arg):
		'Set cam angle [55-80]'
		pos = int(arg)
		pos = 80 - pos * (80-55) / 100.
		self.pub.publish(drive(ord('v'), pos, 1))

	def do_release_cam(self, arg):
		'Release cam'
		self.pub.publish(drive(ord('w'), 0, 1))

	def do_stop(self, arg):
		'Stop motors'
		self.pub.publish(drive(ord('s'), 0, 1))

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
