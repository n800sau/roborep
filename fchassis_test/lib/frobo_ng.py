import time, copy, os, sys, json, pickle
from hmc5883l import hmc5883l
from adxl345 import ADXL345
from l3g4200d import l3g4200
from fchassis_ng import fchassis_ng
from utils import angle_diff
from pids import Pid
from lib.marker import collect_markers

STEP_TIME = 0.01
TICK_MOVE_TIME = 0.5

MIN_DISTANCE = 0.1

class frobo_ng(fchassis_ng):

	def __init__(self, *args, **kwds):
		super(frobo_ng, self).__init__(*args, **kwds)
		self.hit_warn = None
		self.dots = []
		# (int angle): {'lpwr':, 'rpwr':, 'dt':}
		self.memory_fname = os.path.join(os.path.dirname(sys.argv[0]), 'memory.dat')
		self.recover_memory()
		self.compass = hmc5883l(gauss = 4.7, declination = (12, 34))
		self.acc = ADXL345(0x1d)
		self.gyro = l3g4200(1)
		self.read_sensors()

	def add_memory(self, angle, lpwr, rpwr, dt):
		self.memory[int(angle)] = {'angle': angle, 'lpwr': lpwr, 'rpwr': rpwr, 'dt': dt}
		self.save_memory()

	def recover_memory(self):
		self.memory = dict(pickle.load(file(self.memory_fname, 'r')) if os.path.exists(self.memory_fname) else {})

	def save_memory(self):
		pickle.dump(self.memory, file(self.memory_fname, 'w'))

	def find_memory_closest(self, angle):
		rs = None
		prev_k = None
		for k in self.memory.keys():
			if k > angle:
				diff = abs(k - angle)
				if not prev_k is None:
					if abs(prev_k - angle) < diff:
						k = prev_k
				rs = self.memory[k]
				break
			prev_k = k
		return rs

	def read_sensors(self):
		self.state['heading'] = self.compass.heading()
		self.state['acc'] = self.acc.getAxes()

	def current_heading(self):
		self.read_sensors()
		return self.state['heading']

	def heading_diff(self, azim, err=10):
		heading = self.current_heading()
		diff = angle_diff(azim, heading)
		return diff if abs(diff) > err else 0

	def update_state(self):
		rs = super(frobo_ng, self).update_state()
		if rs:
			dot = copy.deepcopy(self.state)
			self.dots.append(dot)
		return rs

	def wait_until_stop(self):
		self.cmd_mstop()
		while not self.is_really_stopped(secs=1):
			pass

	def is_really_stopped(self, secs=1.5):
		h = self.current_heading()
		self.update_state()
		lcount = self.state['lcount']
		rcount = self.state['rcount']
		time.sleep(secs)
		self.update_state()
		return lcount == self.state['lcount'] and \
			rcount == self.state['rcount'] and \
			not self.heading_diff(h, err=2)


	def move_straight(self, fwd=True, max_steps=5, max_secs=1, heading=None, power=50):
		self.cmd_reset_counters()
		pid = Pid(2., 0, 1)
		pid.range(-power, power)
		if heading is None:
			heading = self.current_heading()
		pid.set(heading)
		offset = 0
		try:
			t = time.time()
			while (t + max_secs) > time.time():
				self.update_state()
				lpwr = power + offset
				rpwr = power - offset
				self.dbprint('^%d [%d<>%d] (%d:%d)' % (int(self.current_heading()), lpwr, rpwr, self.state['lcount'], self.state['rcount']))
				if self.state['sonar'] >= 0 and self.state['sonar'] < MIN_DISTANCE:
					self.hit_warn = self.state['sonar']
					self.dbprint('STOP distance=%s' % self.state['sonar'])
					break
				steps = self.steps_counted()
				if steps > max_steps:
					self.dbprint('Max steps reached (%d > %d)' % (steps, max_steps))
					break
				self.cmd_mboth(lpwr, fwd, rpwr, fwd)
				time.sleep(STEP_TIME)
				self.update_state()
				pid.step(input=self.current_heading())
				offset = pid.get()
		finally:
			self.cmd_mstop()

	# return turn degrees
	def tick_move(self, clockwise, min_angle=None, pwr=50):
		rs = 0
		init_t = time.time()
		init_h = self.compass.heading()
		self.dbprint('start h: %d, pwr: %s' % (init_h, pwr))
		try:
			self.cmd_mboth(pwr, clockwise, pwr, not clockwise)
			for i in range(int(TICK_MOVE_TIME/STEP_TIME)):
				st = self.gyro.bus.read_byte_data(self.gyro.address, self.gyro.STATUS_REG)
				if st:
					x,y,z = self.gyro.getDegPerSecAxes()
					h = self.compass.heading()
					if min_angle is None:
						self.dbprint('g:%d %d %d, h:%d' % (x, y, z, h))
					if abs(z) > 15:
						if min_angle is None:
							self.dbprint('it moves')
							break
						else:
							hdiff = abs(angle_diff(h, init_h))
							if hdiff > min_angle:
								self.dbprint('it has moved %d degrees' % hdiff)
								break
				self.update_state()
				time.sleep(STEP_TIME)
		finally:
			self.cmd_mstop()
			self.wait_until_stop()
			h = self.compass.heading()
			self.add_memory(abs(init_h - h), pwr, pwr, time.time() - init_t)
			rs = init_h - h
			self.dbprint('last h: %d, change: %g' % (h, rs))
		return rs

	def tick_left(self, min_angle=None, pwr=40):
		return self.tick_move(False, min_angle=min_angle, pwr=pwr)

	def tick_right(self, min_angle=None, pwr=40):
		return self.tick_move(True, min_angle=min_angle, pwr=pwr)

	def turn(self, azim, err=3, stop_if=None, move_cb=None):
		diff = self.heading_diff(azim, err=err)
		if diff:
#			data = self.find_memory_closest(azim)
			data = None
			if data is None:
				self.turn_in_ticks(azim, err=err, stop_if=stop_if, move_cb=move_cb)
			else:
				self.dbprint("Using memory %s to turn %s degrees" % (data, diff))
				lpwr = data['lpwr']
				rpwr = data['rpwr']
				dt = data['dt'] * (abs(diff) / max(1, data['angle']))
				self.dbprint("Wait for %g" % dt)
				clockwise = diff > 0
				self.cmd_mboth(lpwr, clockwise, rpwr, not clockwise)
				time.sleep(dt)
				self.cmd_mstop()
				self.wait_until_stop()
				diff = self.heading_diff(azim, err=err)
				if diff:
					self.dbprint("Now using turn_in_ticks for the rest %s" % diff)
					self.turn_in_ticks(azim, err=err, stop_if=stop_if, move_cb=move_cb)

	def turn_in_ticks(self, azim, err=3, stop_if=None, move_cb=None):
		min_pwr = 15
		max_pwr = 100
		self.dbprint('start turn h %d' % int(self.current_heading()))
		try:
			last_counts = [self.state['lcount'], self.state['rcount'], 0]
			last_adiff = abs(self.heading_diff(azim, err=err))
			for i in range(360):
				diff = self.heading_diff(azim, err=err)
#				self.dbprint("azim diff=%d (h:%d), acc:%s cnt:%s(pwr:%s)<>%s(pwr%s)" % (
#					diff, self.state['heading'], self.state['acc'],
#					self.state['lcount'], self.state['lpwr'],
#					self.state['rcount'], self.state['rpwr']
#				))
				adiff = abs(diff)
				if adiff < err:
					break
				pwr = min_pwr + (max_pwr - min_pwr) * min(1, (adiff / 180.))
				self.dbprint("power=%g" % pwr)
				if diff > 0:
					change = self.tick_right(min_angle=1, pwr = pwr)
				else:
					change = self.tick_left(min_angle=1, pwr = pwr)
				if abs(change) < 1:
					min_pwr += 5
					if min_pwr > max_pwr - 10:
						min_pwr = max_pwr - 10
				self.wait_until_stop()
				self.db_state()
				if last_counts[0] == self.state['lcount'] and last_counts[1] == self.state['rcount']:
					last_counts[2] += 1
					if last_counts[2] > int(1/STEP_TIME):
						self.dbprint('Stopped moving')
						break
				else:
					if move_cb:
						move_cb(self)
					last_counts = [self.state['lcount'], self.state['rcount'], 0]
				if stop_if:
					if stop_if(self):
						break
		finally:
			self.cmd_mstop()
			self.dbprint('end turn h %d' % int(self.current_heading()))

	def search_around(self, stop_if_cb, clockwise=True, pwr=30, step_angle=10, max_pwr=100):
		min_pwr = pwr
		ih = self.compass.heading()
		last_diff = 0
		growing = True
		n_steps = 360 / step_angle
		for step in range(n_steps):
			change = self.tick_move(clockwise, pwr=pwr, min_angle=step_angle)
			if abs(change) < 1:
				pwr += 5
				if pwr > max_pwr - 10:
					pwr = max_pwr - 10
			else:
				pwr = min_pwr
			self.wait_until_stop()
			h = self.compass.heading()
			adiff = abs(angle_diff(ih, h))
			if growing:
				if adiff < last_diff - 5:
					growing = False
			else:
				if adiff > last_diff + 5:
					break
			last_diff = adiff
			if stop_if_cb(self):
				break

	def stop_if_cb(self, stop_dist):
		n = 10
		self.update_state()
		while self.state['sonar'] < 0 and n > 0:
			time.sleep(0.01)
			self.update_state()
			n -= 1
		return self.state['sonar'] > stop_dist if self.state['sonar'] >= 0 else False

	def find_distance(self, min_dist=100, clockwise=True):
		self.stop_if_cb(1)
		# 10 attempt to turn
		i = 10
		while self.state['sonar'] < min_dist and i>0:
			self.turn_in_ticks(self.current_heading() + (45 if clockwise else -45), stop_if=lambda c: c.stop_if_cb(min_dist))
			i -= 1

	def find_left_minimum(self, fwd=True):
		return self.find_pwr_minimum('left', fwd)

	def find_right_minimum(self, fwd=True):
		return self.find_pwr_minimum('right', fwd)

	def find_pwr_minimum(self, motor, fwd):
		rs = None
		func = self.cmd_mright if motor[0] == 'r' else self.cmd_mleft
		try:
			for pwr in range(100):
				func(pwr, fwd)
				for i in range(int(0.2 / STEP_TIME)):
					time.sleep(STEP_TIME)
					st = self.gyro.bus.read_byte_data(self.gyro.address, self.gyro.STATUS_REG)
					if st:
						x,y,z = self.gyro.getDegPerSecAxes()
						if abs(z) > 10:
							self.dbprint('it moves')
							rs = pwr
							break
				if not rs is None:
					break
		finally:
			self.cmd_mstop()
		return rs


	def search_marker(self, r, clockwise=True, marker_id=None):

		def collect_markers_cb(r, marker_id):
			class do_it:

				def __init__(self):
					self.i = 0;
					self.target_loc = None

				def __call__(self, c):
					rs = False
					fname = os.path.join(os.path.expanduser('~/public_html'), 'pic%d.jpg' % self.i)
					markers = collect_markers(r, fpath=fname)
					if markers:
						c.dbprint('FOUND %d markers:' % len(markers))
						for m in markers:
							if marker_id is None:
								c.dbprint('\t%s' % (m['id'], ))
							else:
								c.dbprint('\t%s vs %s' % (m['id'], marker_id))
							#	os.system('espeak "%s"' % ' '.join([c for c in str(m['id'])]))
							if marker_id and m['id'] == marker_id:
								self.target_loc = m
								rs = True
								break
						self.i += 1
					return rs

			return do_it()

		cb = collect_markers_cb(r, marker_id)
		self.search_around(cb, clockwise=clockwise)
		return cb.target_loc
