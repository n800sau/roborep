#!/usr/bin/env python

import curses, time
from robec import robec

class irobec:

	mmenu = (
			{
				'title': 'Item 1',
				'key': '1',
				'cb': 'func1',
			},
			{
				'title': 'Item 2',
				'key': '2',
				'cb': 'func2',
			},
			{
				'title': 'Item 3',
				'key': '3',
				'cb': 'func3',
			},
			{
				'title': 'Item 4',
				'key': '4',
				'cb': 'func4',
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
			self.stdscr.addstr(self.maxy - 10, 0, '%.2f' % reply['head'])

	def func1(self):
		reply = self.c.read_json()
		self.stdscr.addstr(self.maxy - 3, 0, 'Pos 1')

	def func2(self):
		self.stdscr.addstr(self.maxy - 3, 0, 'Pos 2')

	def func3(self):
		self.stdscr.addstr(self.maxy - 3, 0, 'Pos 3')

	def func4(self):
		self.stdscr.addstr(self.maxy - 3, 0, 'Pos 4')

	def update(self):

		y = self.maxy - len(self.mmenu) - 4

		for item in self.mmenu:
			self.stdscr.addstr(y, 0, '%s: %s' % (item['key'], item['title']))
			y += 1

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
					if ord(item['key']) == c:
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
