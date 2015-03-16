#!/usr/bin/env python

import curses, time
from robec import robec

class irobec:

	mmenu = (
			{
				'title': 'L',
				'key': 'A',
				'cb': 'turn_left',
				'y': 1,
				'x': 0,
			},
			{
				'title': 'R',
				'key': 'D',
				'cb': 'turn_right',
				'y': 1,
				'x': 2,
			},
			{
				'title': 'F',
				'key': 'W',
				'cb': 'step_forward',
				'y': 0,
				'x': 1,
			},
			{
				'title': 'B',
				'key': 'S',
				'cb': 'step_back',
				'y': 2,
				'x': 1,
			},
		)

	def __init__(self):
		self.c = robec()
		self.c.connect('192.168.1.96', 23)

		self.stdscr = curses.initscr()

	def init(self):
		curses.noecho()
		curses.cbreak()
		curses.curs_set(0)

		self.stdscr.scrollok(False)
		self.stdscr.keypad(True)
		self.stdscr.timeout(1);

		self.maxy,self.maxx = self.stdscr.getmaxyx()
		self.update()

	def reset(self):
		self.stdscr.refresh()
		curses.endwin()

	def show_sensors(self):
		self.c.send_command("sensors")
		reply = self.c.read_json()
		if reply:
			self.stdscr.addstr(self.maxy - 11, 0, time.strftime('%d.%m.%y %H:%M:%S'))
			self.stdscr.addstr(self.maxy - 10, 0, '%.2f' % reply['head'])

	def turn_left(self):
		self.c.send_command("turn_left")
		self.stdscr.addstr(self.maxy - 3, 0, 'L')

	def turn_right(self):
		self.c.send_command("turn_right")
		self.stdscr.addstr(self.maxy - 3, 0, 'R')

	def step_forward(self):
		self.c.send_command("step_forward")
		self.stdscr.addstr(self.maxy - 3, 0, 'F')

	def step_back(self):
		self.c.send_command("step_back")
		self.stdscr.addstr(self.maxy - 3, 0, 'B')

	def update(self):

		y = self.maxy - len(self.mmenu) - 4

		for item in self.mmenu:
			self.stdscr.addstr(y + item['y'], 0 + item['x'], item['title'])

		self.stdscr.refresh()

	def run(self):
		t = time.time()
		while True:
			c = self.stdscr.getch()
			if c > 0:
#				print 'c=',c
				if c in (ord('q'), ord('Q')):
					self.stdscr.clear()
					break  # Exit the while()
				for item in self.mmenu:
					if c in (ord(item['key'].lower()), ord(item['key'].upper())) :
#						try:
						getattr(self, item['cb'])()
#						except Exception, e:
#							self.stdscr.addstr(self.maxy - 3, 0, str(e))
						break
				self.update()
			else:
				self.c.spin()
				if time.time() - t > 1:
					t = time.time()
					self.show_sensors()

if __name__ == '__main__':

	r = irobec()
	try:
		r.init()
		r.run()
	finally:
		r.reset()
