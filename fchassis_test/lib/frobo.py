import time
from hmc5883l import hmc5883l
from adxl345 import ADXL345
from l3g4200d import l3g4200
from fchassis import fchassis, ENC_STEP, ENCODER_L, ENCODER_R
from utils import angle_diff
from pids import Pid

STEP_TIME = 0.01

class frobo(fchassis):

	def __init__(self, *args, **kwds):
		self.hit_warn = None
		self.dots = {ENCODER_L: [], ENCODER_R: []}
		self.compass = hmc5883l(gauss = 4.7, declination = (12, 34))
		self.acc = ADXL345(0x1d)
		self.gyro = l3g4200(1)
		self.read_sensors()
		super(frobo, self).__init__(*args, **kwds)

	def read_sensors(self):
		self.last_heading = self.compass.heading()
		self.last_acc = self.acc.getAxes()

	def current_heading(self):
		self.read_sensors()
		return self.last_heading

	def heading_diff(self, azim, err=10):
		heading = self.current_heading()
		diff = angle_diff(azim, heading)
		return diff if abs(diff) > err else 0

	def count_change(self, pin, step, t, dt):
		super(frobo, self).count_change(pin, step, t, dt)
		dot = {
			'heading': self.last_heading,
			'acc': self.last_acc,
			'step': step,
			'dt': dt,
			'v': ENC_STEP * step / dt,
			'dist': self.curr_dist,
			't': t,
		}
		if self.hit_warn:
			dot['hit_warn'] = self.hit_warn
			self.hit_warn = None
		self.dots[pin].append(dot)

	# turn in steps
	def turn_in_steps(self, azim, err=5, stop_if=None, move_cb=None, power_offsets=None):
		POWER_STEP = 2
		MOVE_STEP_TIME = 0.3
		self.wait_until_stop()
		if power_offsets is None:
			power_offsets = self.pwr_offsets()
		lpwr = power_offsets['lmin']
		rpwr = power_offsets['rmin']
		self.reset_counters()
		self.dbprint('start turn h %d' % int(self.current_heading()))
		try:
			last_counts = [self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_R]['count'], 0]
			last_adiff = abs(self.heading_diff(azim, err=err))
			for i in range(100):
				diff = self.heading_diff(azim, err=err)
				self.dbprint("azim diff=%d (h:%d), acc:%s cnt:%s(%s)<>%s(%s)" % (
					diff, self.last_heading, self.last_acc,
					self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_L]['pwr'],
					self.enc_data[ENCODER_R]['count'], self.enc_data[ENCODER_R]['pwr']
				))
				if diff > 0:
					ldir = True
					rdir = False
				else:
					ldir = False
					rdir = True
				adiff = abs(diff)
				if adiff < err:
					# reverse
					self.dbprint('Break!!!')
					self.left_move(not ldir, 100)
					self.right_move(not rdir, 100)
					time.sleep(0.4)
					break
				if last_adiff < adiff:
					# too fast, slow down
					lpwr -= POWER_STEP
					rpwr -= POWER_STEP
					if lpwr <= 0:
						lpwr = 1
					if rpwr <= 0:
						rpwr = 1
				self.left_move(ldir, lpwr)
				self.right_move(rdir, rpwr)
				time.sleep(MOVE_STEP_TIME)
				self.stop()
				self.wait_until_stop()
				self.db_state()
				if last_counts[0] == self.enc_data[ENCODER_L]['count'] and last_counts[1] == self.enc_data[ENCODER_R]['count']:
					last_counts[2] += 1
					if last_counts[2] > int(1/STEP_TIME):
						self.dbprint('Stopped moving')
						lpwr += POWER_STEP
						rpwr += POWER_STEP
						if lpwr >= 100 or rpwr >= 100:
							break
						last_counts[2] = 0
				else:
					if move_cb:
						move_cb(self)
					last_counts = [self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_R]['count'], 0]
				if stop_if:
					if stop_if(self):
						break
		finally:
			self.stop()
			self.dbprint('end turn h %d' % int(self.current_heading()))

	def turn_in_ticks(self, azim, err=3, stop_if=None, move_cb=None):
		self.wait_until_stop()
		self.reset_counters()
		self.dbprint('start turn h %d' % int(self.current_heading()))
		try:
			last_counts = [self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_R]['count'], 0]
			last_adiff = abs(self.heading_diff(azim, err=err))
			for i in range(360):
				diff = self.heading_diff(azim, err=err)
				self.dbprint("azim diff=%d (h:%d), acc:%s cnt:%s(%s)<>%s(%s)" % (
					diff, self.last_heading, self.last_acc,
					self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_L]['pwr'],
					self.enc_data[ENCODER_R]['count'], self.enc_data[ENCODER_R]['pwr']
				))
				adiff = abs(diff)
				if adiff < err:
					break
				if diff > 0:
					self.tick_right(min_angle=1)
				else:
					self.tick_left(min_angle=1)
				self.wait_until_stop()
				self.db_state()
				if last_counts[0] == self.enc_data[ENCODER_L]['count'] and last_counts[1] == self.enc_data[ENCODER_R]['count']:
					last_counts[2] += 1
					if last_counts[2] > int(1/STEP_TIME):
						self.dbprint('Stopped moving')
						break
				else:
					if move_cb:
						move_cb(self)
					last_counts = [self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_R]['count'], 0]
				if stop_if:
					if stop_if(self):
						break
		finally:
			self.stop()
			self.dbprint('end turn h %d' % int(self.current_heading()))

	def turn(self, azim, err=5, stop_if=None, move_cb=None, power_offsets=None):
		if power_offsets is None:
			power_offsets = self.pwr_offsets()
		lpwr = power_offsets['lmin']
		rpwr = power_offsets['rmin']
		self.reset_counters()
		self.dbprint('start turn h %d' % int(self.current_heading()))
		try:
			last_counts = [self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_R]['count'], 0]
			for i in range(int(10/STEP_TIME)):
				diff = self.heading_diff(azim, err=err)
				self.dbprint("azim diff=%d (h:%d), acc:%s cnt:%s(%s)<>%s(%s)" % (
					diff, self.last_heading, self.last_acc,
					self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_L]['pwr'],
					self.enc_data[ENCODER_R]['count'], self.enc_data[ENCODER_R]['pwr']
				))
				if diff > 0:
					ldir = True
					rdir = False
				else:
					ldir = False
					rdir = True
				adiff = abs(diff)
				if adiff < err:
					# reverse
					self.dbprint('Break!!!')
					self.left_move(not ldir, 100)
					self.right_move(not rdir, 100)
					time.sleep(0.4)
					break
				self.left_move(ldir, lpwr)
				self.right_move(rdir, rpwr)
				self.db_state()
				time.sleep(STEP_TIME)
				if last_counts[0] == self.enc_data[ENCODER_L]['count'] and last_counts[1] == self.enc_data[ENCODER_R]['count']:
					last_counts[2] += 1
					if last_counts[2] > int(1/STEP_TIME):
						self.dbprint('Stopped moving')
						lpwr += 2
						rpwr += 2
						if lpwr >= 100 or rpwr >= 100:
							break
						last_counts[2] = 0
				else:
					if move_cb:
						move_cb(self)
					last_counts = [self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_R]['count'], 0]
				if stop_if:
					if stop_if(self):
						break
		finally:
			self.stop()
			self.dbprint('end turn h %d' % int(self.current_heading()))

	def turn_fast(self, azim, err=5, stop_if=None, move_cb=None):
		self.reset_counters()
		min_pwr = 10
		pwr = 100
		self.dbprint('start turn h %d' % int(self.current_heading()))
		try:
			# maximum ~ 10 sec
			last_counts = [self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_R]['count'], 0]
			for i in range(int(10/STEP_TIME)):
				diff = self.heading_diff(azim, err=err)
				self.dbprint("azim diff=%d (h:%d), acc:%s cnt:%s(%s)<>%s(%s)" % (
					diff, self.last_heading, self.last_acc,
					self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_L]['pwr'],
					self.enc_data[ENCODER_R]['count'], self.enc_data[ENCODER_R]['pwr']
				))
				if diff > 0:
					ldir = True
					rdir = False
				else:
					ldir = False
					rdir = True
				adiff = abs(diff)
				if adiff < 30:
					p = pwr * 0.3
				elif adiff < 50:
					p = pwr * 0.5
				elif adiff < 100:
					p = pwr * 0.7
				else:
					p = pwr
				# to make p bigger if it is too small
				if p < min_pwr:
					p = min_pwr
				if adiff < err:
					self.dbprint('Reversing...')
					self.left_move(not ldir, p)
					self.right_move(not rdir, p)
					pwr /= 2
					time.sleep(0.5)
					adiff = abs(self.heading_diff(azim, err=err))
					if adiff < err:
						break
				else:
					self.left_move(ldir, p)
					self.right_move(rdir, p)
				self.db_state()
				time.sleep(STEP_TIME)
				if last_counts[0] == self.enc_data[ENCODER_L]['count'] and last_counts[1] == self.enc_data[ENCODER_R]['count']:
					last_counts[2] += 1
					if last_counts[2] > int(1/STEP_TIME):
						self.dbprint('Stopped moving')
#						if adiff > err:
#							min_pwr += 1
#							if min_pwr > 30:
#								min_pwr = 30
#						else:
						break
				else:
					if move_cb:
						move_cb(self)
					last_counts = [self.enc_data[ENCODER_L]['count'], self.enc_data[ENCODER_R]['count'], 0]
				if stop_if:
					if stop_if(self):
						break
		finally:
			self.stop()
			self.dbprint('end turn h %d' % int(self.current_heading()))

	def turn_pid(self, azim, vel=0.2, err=5):
		self.reset_counters()
		pwr = 30
		lpwr = 0
		lpid = Pid(10., 1, 2)
		lpid.range(-pwr, pwr)
		lsz = len(self.dots[ENCODER_L])
		rpwr = 0
		rpid = Pid(10., 1, 2)
		rpid.range(-pwr, pwr)
		rsz = len(self.dots[ENCODER_R])
		self.dbprint('%d' % int(self.current_heading()))
		try:
			diff = self.heading_diff(azim, err=err)
			if diff:
				if diff > 0:
					ldir = True
					rdir = False
				else:
					ldir = False
					rdir = True
				lpid.set(vel * (1 if ldir else -1))
				rpid.set(vel * (1 if rdir else -1))
				# maximum ~ 100 sec
				for i in range(int(100/STEP_TIME)):
					self.dbprint("lpwr=%s, rpwr=%s" % (pwr-lpwr, pwr-rpwr))
					self.left_move(ldir, pwr - lpwr)
					self.right_move(rdir, pwr - rpwr)
					clsz = len(self.dots[ENCODER_L])
					crsz = len(self.dots[ENCODER_R])
					if lsz < clsz and rsz < crsz:
						lsz = clsz
						lvel = self.dots[ENCODER_L][-1]['v']
						ldt = self.dots[ENCODER_L][-1]['dt']
						lpid.step(dt=ldt, input=lvel)
						lpwr = lpid.get()
						rsz = crsz
						rvel = self.dots[ENCODER_R][-1]['v']
						rdt = self.dots[ENCODER_R][-1]['dt']
						rpid.step(dt=rdt, input=rvel)
						rpwr = rpid.get()
						self.dbprint("lvel=%s, rvel=%s" % (lvel, rvel))
					diff = self.heading_diff(azim, err=0)
					self.dbprint("azim diff=%d (h:%d), acc:%s" % (diff, self.last_heading, self.last_acc))
					if not diff:
						break
					self.db_state()
					time.sleep(STEP_TIME)
		finally:
			self.stop()
			self.dbprint('%d' % int(self.current_heading()))


	def fwd_straight(self, max_steps=5, max_secs=1, heading=None, power=50, power_offsets=None):
		if power_offsets is None:
			power_offsets = self.pwr_offsets()
		self.reset_counters()
		pid = Pid(2., 0, 1)
		pid.range(-power, power)
		if heading is None:
			heading = self.current_heading()
		pid.set(heading)
		offset = 0
		try:
			for i in range(int(max_secs/STEP_TIME)):
				lpwr = power + offset
				rpwr = power - offset
				self.dbprint('^%d [%d<>%d] (%d:%d)' % (int(self.current_heading()), lpwr, rpwr, self.mleft['count'], self.mright['count']))
				steps = self.steps_counted()
				if steps > max_steps:
					self.dbprint('Max steps reached (%d > %d)' % (steps, max_steps))
					break
				self.left_move(True, power_offsets['lmin'] + lpwr)
				self.right_move(True, power_offsets['rmin'] + rpwr)
				self.db_state()
				time.sleep(STEP_TIME)
				pid.step(input=self.current_heading())
				offset = pid.get()
				if self.curr_dist > 0 and self.curr_dist < 20:
					self.hit_warn = self.curr_dist
					self.dbprint('STOP distance=%s' % self.curr_dist)
					break
		finally:
			self.stop()
			self.dbprint('%d' % int(self.current_heading()))

	def stop_if_cb(self, stop_dist):
		n = 10
		self.update_dist()
		while (self.curr_dist == [127] or self.curr_dist == 0) and n > 0:
			time.sleep(0.01)
			self.update_dist()
			n -= 1
		return self.curr_dist > stop_dist if self.curr_dist != [127] and self.curr_dist != 0 else False

	def find_distance(self, min_dist=100, clockwise=True):
		self.stop_if_cb(1)
		# 10 attempt to turn
		i = 10
		while self.curr_dist < min_dist and i>0:
			self.turn(self.current_heading() + (45 if clockwise else -45), stop_if=lambda c: c.stop_if_cb(min_dist))
			i -= 1

	def search_around(self, stop_if_cb, clockwise=True):
		offs = {True: self.pwr_offsets(fwd_dir=True), False: self.pwr_offsets(fwd_dir=False)}
		ldir,rdir = (True, False) if clockwise else (False, True)
		ih = self.compass.heading()
		last_diff = 0
		growing = True
		while True:
			self.left_move(ldir, offs[ldir]['lmin'] + 20)
			self.right_move(rdir, offs[rdir]['rmin'] + 20)
			time.sleep(0.2)
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

	def search_around1(self, stop_if_cb, clockwise=True):
		# 36 attempt to turn 10 degree
		for i in range(36):
			self.turn(self.current_heading() + (10 if clockwise else -10))
			self.wait_until_stop()
			if stop_if_cb(self):
				break

	def find_left_minimum(self, fwd_dir=True):
		return self.find_pwr_minimum(ENCODER_L, fwd_dir)

	def find_right_minimum(self, fwd_dir=True):
		return self.find_pwr_minimum(ENCODER_R, fwd_dir)

	def find_pwr_minimum(self, enc_index, fwd_dir):
		count = self.enc_data[enc_index]['count']
		h = self.current_heading()
		for pwr in range(0, 100, 2):
			self.dbprint('%s: %d, h: %d' % (self.enc_data[enc_index]['name'], pwr, self.current_heading()))
			if enc_index == ENCODER_R:
				self.right_move(fwd_dir, pwr)
			else:
				self.left_move(fwd_dir, pwr)
			time.sleep(0.2)
			self.db_state()
			if count != self.enc_data[enc_index]['count'] or self.heading_diff(h, err=2):
				break
		self.stop()
		return pwr

	def wait_until_stop(self):
		self.stop()
		while not self.is_really_stopped(secs=1):
			pass

	def is_really_stopped(self, secs=1.5):
		h = self.current_heading()
		lcount = self.enc_data[ENCODER_L]['count']
		rcount = self.enc_data[ENCODER_R]['count']
		self.db_state()
		time.sleep(secs)
		return lcount == self.enc_data[ENCODER_L]['count'] and \
			rcount == self.enc_data[ENCODER_R]['count'] and \
			not self.heading_diff(h, err=2)

	def pwr_offsets(self, fwd_dir=True):
		lpwr = self.find_left_minimum(fwd_dir=fwd_dir)
		while not self.is_really_stopped():
			pass
		rpwr = self.find_right_minimum(fwd_dir=fwd_dir)
		for i in range(10):
			if self.is_really_stopped():
				break
		return {'lmin': lpwr, 'rmin': rpwr}

	def tick_move(self, clockwise, min_angle=None, pwr=40):
		init_h = self.compass.heading()
		self.dbprint('init h: %d' % init_h)
		try:
			self.left_move(clockwise, pwr)
			self.right_move(not clockwise, pwr)
			for i in range(100):
				st = self.gyro.bus.read_byte_data(self.gyro.address, self.gyro.STATUS_REG)
				if st:
					x,y,z = self.gyro.getDegPerSecAxes()
					h = self.compass.heading()
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
				time.sleep(0.001)
		finally:
			self.stop()
			time.sleep(2)
			h = self.compass.heading()
			self.dbprint('last h: %d, change: %g' % (h, init_h - h))

	def tick_left(self, min_angle=None, pwr=40):
		self.tick_move(False, min_angle=min_angle, pwr=pwr)

	def tick_right(self, min_angle=None, pwr=40):
		self.tick_move(True, min_angle=min_angle, pwr=pwr)

