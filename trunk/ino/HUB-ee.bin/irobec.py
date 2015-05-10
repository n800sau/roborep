#!/usr/bin/env python

import curses, time, math, sys, traceback, struct
from robec import robec
import redis
import json
import pycmds

# 0.1 is average time for reply
SENSORS_SHOW_PERIOD = 0.1

class irobec:

	mmenu = (
			{
				'title': 'L',
				'key': 'A',
				'cb': 'cmd_turn_left',
				'y': 1,
				'x': 0,
			},
			{
				'title': 'R',
				'key': 'D',
				'cb': 'cmd_turn_right',
				'y': 1,
				'x': 2,
			},
			{
				'title': 'F',
				'key': 'W',
				'cb': 'cmd_step_forward',
				'y': 0,
				'x': 1,
			},
			{
				'title': 'B',
				'key': 'S',
				'cb': 'cmd_step_back',
				'y': 2,
				'x': 1,
			},
			{
				'title': 'R-RST',
				'key': 'R',
				'cb': 'cmd_reset',
				'y': 1,
				'x': 5,
			},
			{
				'title': 'Z-res enc',
				'key': 'Z',
				'cb': 'cmd_reset_encoders',
				'y': 2,
				'x': 5,
			},
			{
				'title': 'M-cal mot',
				'key': 'M',
				'cb': 'cmd_calibrate_motors',
				'y': 3,
				'x': 5,
			},
			{
				'title': 'C-zacc',
				'key': 'C',
				'cb': 'cmd_set_acc_zero',
				'y': 4,
				'x': 5,
			},
			{
				'title': 'P-spin',
				'key': 'P',
				'cb': 'cmd_spinning',
				'y': 4,
				'x': 12,
			},
		)

	def __init__(self):
		self.c = robec()
		self.c.connect('192.168.1.97', 23)
		self.wait4sensors = False
		self.x = 0
		self.y = 0
		self.r = redis.Redis()
		self.r.delete('xy')
		self.hit_time = None

	def init(self, stdscr=None):
		if stdscr is None:
			self.stdscr = curses.initscr()
		else:
			self.stdscr = stdscr
		curses.use_default_colors()
		curses.noecho()
		curses.cbreak()
		curses.curs_set(0)

		self.stdscr.scrollok(False)
		self.stdscr.keypad(True)
		self.stdscr.timeout(1);

		self.maxy,self.maxx = self.stdscr.getmaxyx()
		self.update()

	def dbprint(self, text):
		print >>sys.__stderr__, text

	def reset(self):
		self.stdscr.refresh()
		curses.endwin()

	def cmd_show_sensors(self):
		self.wait4sensors = True
		self.c.debug = False
		try:
			r = {
				'V': None,
				'heading': None,
				'dist': None,
				'acc_x_max': None,
				'LC': None,
				'RC': None,
				'Lcoef': None,
				'Rcoef': None,
				'T': None,
				'vects': [],
			}
			self.c.send_command(pycmds.C_STATE)
			for i in range(100):
				data = self.c.read_bin()
				self.dbprint('Data=%s' % data)
				if data:
					if data['cmd'] == pycmds.R_HIT_1F:
						if data['vals'][0]:
							self.hit_time = time.time()
						elif self.hit_time:
							if time.time() - self.hit_time > 5:
								self.hit_time = None
					elif data['cmd'] == pycmds.R_ACC_3F:
						r['acc_x'],r['acc_y'],r['acc_z'] = data['vals']
					elif data['cmd'] == pycmds.R_ACCMAX_3F:
						r['acc_x_max'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_HEADING_1F:
						r['heading'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_DISTANCE_1F:
						r['dist'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_MCOUNTS_2F:
						r['LC'],r['RC'] = data['vals']
					elif data['cmd'] == pycmds.R_MCOEF_2F:
						r['Lcoef'],r['Rcoef'] = data['vals']
					elif data['cmd'] == pycmds.R_TEMPERATURE_1F:
						r['T'] = data['vals'][0]
					elif data['cmd'] == pycmds.R_VECTOR_3F:
						r['vects'].append({'lC': data['vals'][0], 'rC': data['vals'][1], 'h': data['vals'][2]})
					elif data['cmd'] == pycmds.R_END:
						#self.dbprint('Done')
						break
			if r:
				self.stdscr.clear()
				y = 20
				self.stdscr.addstr(self.maxy - y, 0, time.strftime('%d.%m.%y %H:%M:%S'))
				y -= 1
				if not r['V'] is None:
					self.stdscr.addstr(self.maxy - y, 0, 'V:%.3f' % (r['V']/1000.))
				y -= 1
				if not (r['heading'] is None or r['dist'] is None):
					self.stdscr.addstr(self.maxy - y, 0, '%.2f -> %d (%g) %s' % (r['heading'], r['dist'], r['acc_x_max'], ('hit!' if self.hit_time else '')))
				y -= 1
				if not (r['LC'] is None or r['RC'] is None):
					self.stdscr.addstr(self.maxy - y, 0, ' %4d <> %4d' % (r['LC'], r['RC']))
				y -= 1
				if not (r['Lcoef'] is None or r['Rcoef'] is None):
					self.stdscr.addstr(self.maxy - y, 0, '[%.2f <> %.2f]' % (r['Lcoef'], r['Rcoef']))
				y -= 1
				if not r['T'] is None:
					self.stdscr.addstr(self.maxy - y, 0, 'T:%d' % r['T'])
				y -= 1
				# north is X
#				self.dbprint('vectors=%s\n' % r['vects'])
				for v in r['vects']:
#					self.dbprint('v=%s\n' % v)
					dist = (v['rC'] + v['lC']) / 2.
					angl = v['h']
					dx = math.cos(math.radians(angl))
					dy = math.sin(math.radians(angl))
					self.x += dx * dist
					self.y += dy * dist
					v['x'] = self.x
					v['y'] = self.y
					self.r.rpush('xy', json.dumps(v))
					self.r.rpush('accvec', json.dumps({'x': r['acc_x'], 'y': r['acc_y'], 'z': r['acc_z']}))
					self.stdscr.addstr(self.maxy - y, 0, '%.1f, %.1f' % (self.x, self.y))
		except:
			self.dbprint(traceback.format_exc())
		finally:
			self.c.debug = True
			self.wait4sensors = False

	def wait_reply(self):
		for i in range(100):
			data = self.c.read_bin()
			if data and data['cmd'] == pycmds.R_OK_0:
				self.stdscr.addstr(self.maxy - 3, 0, 'OK')
				break
			time.sleep(0.1)

	def cmd_turn(self, rad):
		self.c.send_command(cmd.C_TURN2HEAD, struct.pack('<H', rad))
		self.stdscr.addstr(self.maxy - 3, 0, 'TR')

	def cmd_turn_left(self):
		self.c.send_command(pycmds.C_TLEFT)
		self.stdscr.addstr(self.maxy - 3, 0, 'L')

	def cmd_stop(self):
		self.c.send_command(pycmds.C_STOP)
		self.stdscr.addstr(self.maxy - 3, 0, '-')

	def cmd_turn_right(self):
		self.c.send_command(pycmds.C_TRIGHT)
		self.stdscr.addstr(self.maxy - 3, 0, 'R')

	def cmd_step_forward(self):
		self.c.send_command(pycmds.C_FORWARD)
		self.stdscr.addstr(self.maxy - 3, 0, 'F')

	def cmd_step_back(self):
		self.c.send_command(pycmds.C_BACK)
		self.stdscr.addstr(self.maxy - 3, 0, 'B')

	def cmd_reset(self):
		self.c.send_at("IORST")
		self.stdscr.addstr(self.maxy - 3, 0, 'X')

	def cmd_reset_encoders(self):
		self.c.send_command(pycmds.C_RESCNT)
		self.stdscr.addstr(self.maxy - 3, 0, 'Z')
		self.x = self.y = 0

	def cmd_calibrate_motors(self):
		self.c.send_command(pycmds.C_MCALIB)
		self.stdscr.addstr(self.maxy - 3, 0, 'M')

	def cmd_set_acc_zero(self):
		self.c.send_command(pycmds.C_SETACC)
		self.stdscr.addstr(self.maxy - 3, 0, 'C')

	def cmd_spinning(self):
		self.c.send_command(pycmds.C_SPIN)
		self.stdscr.addstr(self.maxy - 3, 0, 'SP')

	def update(self):

		y = self.maxy - len(self.mmenu) - 4

		for item in self.mmenu:
			self.stdscr.addstr(y + item['y'], 0 + item['x'], item['title'])

		self.stdscr.refresh()

	def run(self):
		self.c.send_at("UPRI")
		t = time.time()
		while True:
			c = self.stdscr.getch()
			if c > 0:
				if c in (ord('q'), ord('Q')):
					self.stdscr.clear()
					break  # Exit the while()
				for item in self.mmenu:
					if c in (ord(item['key'].lower()), ord(item['key'].upper())) :
						getattr(self, item['cb'])()
						#self.wait_reply()
						break
				self.update()
			else:
				try:
					cmd = self.c.check_rcommands()
					if cmd:
						self.dbprint('cmd=%s' % (cmd,))
						if cmd[0] == 'command':
							cmd = json.loads(cmd[1])
							cmdfunc = getattr(self, 'cmd_' + cmd['name'], None)
							if cmdfunc:
								cmdfunc(**cmd.get('params', {}))
					self.c.spin()
					tdiff = time.time() - t
					if tdiff > SENSORS_SHOW_PERIOD and not self.wait4sensors:
						t = time.time()
						self.cmd_show_sensors()
						self.stdscr.addstr(0, 0, '%.2f' % tdiff)
						self.update()
				except:
					self.dbprint(traceback.format_exc())

def tapp(stdscr):
	r = irobec()
	try:
		r.init(stdscr)
		r.run()
	finally:
		r.reset()

if __name__ == '__main__':

	curses.wrapper(tapp)

