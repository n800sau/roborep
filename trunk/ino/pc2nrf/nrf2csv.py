#!/usr/bin/env python

import cmd, sys, os, serial, atexit, time, csv

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'include', 'swig'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lib_py'))

import libcommon_py as common
from conf_ino import configure

class SerReader:

	def __init__(self):
		cfg = configure().as_dict('serial')
		self.s = serial.Serial(cfg['serial_port'], int(cfg.get('baud_rate', 9600)), timeout=5, interCharTimeout=1)
		self.reply_mode = False
		self.replylist = []
		self.clean_replylist()

	def clean_replylist(self):
		self.final_replylist = {}
		for ndx in {common.REPLY_MARKER, common.SERVO_STATE_MARKER, common.CONTROLLER_STATE_MARKER}:
			self.final_replylist[ndx] = []
		self.has_reply = False

	def read_replylist(self):
		rs = self.final_replylist
		self.clean_replylist()
		return rs

	def readline(self):
		return self.s.readline()

	def write(self, line):
		self.s.write(line)

	def idle(self):
		if self.s.inWaiting():
			line = self.readline()
#			print line
#			print >>sys.stderr, '%s, %s, %s' % (line, common.REPLY_START_MARKER, line == common.REPLY_START_MARKER)
#			print >>sys.stderr, self.final_replylist
			if self.reply_mode:
				if line.strip() == common.REPLY_END_MARKER:
					self.final_replylist[common.REPLY_MARKER] += self.replylist
					self.has_reply = True
					self.replylist = []
					self.reply_mode = False
				else:
					#print >>sys.stderr, line
					self.replylist.append(line)
			else:
				if line.startswith(common.REPLY_MARKER):
					#print >>sys.stderr, line
					self.final_replylist[common.REPLY_MARKER].append(line[len(common.REPLY_MARKER):])
					self.has_reply = True
				elif line.strip() == common.REPLY_START_MARKER:
					self.reply_mode = True
				elif line.startswith(common.SERVO_STATE_MARKER):
					self.final_replylist[common.SERVO_STATE_MARKER].append(line[len(common.SERVO_STATE_MARKER):])
					self.has_reply = True
				elif line.startswith(common.CONTROLLER_STATE_MARKER):
					self.final_replylist[common.CONTROLLER_STATE_MARKER].append(line[len(common.CONTROLLER_STATE_MARKER):])
					self.has_reply = True

if __name__ == '__main__':
	files = {}
	sreader = SerReader()
	while True:
		sreader.idle()
		if sreader.has_reply:
			for k,vl in sreader.read_replylist().items():
				if vl:
					if k not in files:
						files[k] = {}
						files[k]['file'] = file(k + '.csv', 'a')
						files[k]['csv'] = csv.writer(files[k]['file'])
					for r in vl:
						files[k]['csv'].writerow([time.strftime('%d.%m.%Y %H:%M:%S')] + [s.strip() for s in r.strip().split('\t')])
					files[k]['file'].flush()
#					print k, '=', '\n'.join(vl)
