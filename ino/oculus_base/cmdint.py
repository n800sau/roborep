#!/usr/bin/env python

import cmd, sys, os, serial, atexit, readline, socket, time

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

from forklift import run_process, terminate_all
import libcommon_py as common
from oculus_intf import OculusIntf

class OculusShell(cmd.Cmd, OculusIntf):
	intro = 'Welcome to the oculus shell.	Type help or ? to list commands.\n'
	prompt = '(oculus) '

	def __init__(self):
		cmd.Cmd.__init__(self)
		OculusIntf.__init__(self)
		self.file = None
		self.logfile = None

	def write_log(self, line):
		if not self.logfile:
			self.logfile = file('oculus_base.log', 'a+')
		self.logfile.write('%s\n' % line.rstrip())
		self.logfile.flush()

	# ----- oculus commands -----

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

