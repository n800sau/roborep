import time
from hmc5883l import hmc5883l
from adxl345 import ADXL345
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

	def turn(self, azim, err=5, stop_if=None, move_cb=None):
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

	def turn1(self, azim, vel=0.2, err=5):
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


	def fwd_straightly(self, max_steps=5, max_secs=1, heading=None, power=50):
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
				self.left_move(True, lpwr)
				self.right_move(True, rpwr)
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
