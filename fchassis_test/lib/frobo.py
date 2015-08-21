import time
from hmc5883l import hmc5883l
from adxl345 import ADXL345
from fchassis import fchassis, ENC_STEP, ENCODER_L, ENCODER_R
from utils import angle_diff
from pids import Pid

STEP_TIME = 0.01

class frobo(fchassis):

	def __init__(self, *args, **kwds):
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
			't': t,
		}
		self.dots[pin].append(dot)

	def turn(self, azim, vel=0.2, err=5):
		self.reset_counters()
		pwr = 30
		lpwr = 0
		lpid = Pid(20., 1., 1.)
		lpid.range(-pwr, pwr)
		lsz = len(self.dots[ENCODER_L])
		rpwr = 0
		rpid = Pid(20., 1., 1.)
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
					diff = self.heading_diff(azim, err=err)
					self.dbprint("azim diff=%d (h:%d)" % (diff, self.last_heading))
					if not diff:
						break
					self.db_state()
					time.sleep(STEP_TIME)
		finally:
			self.stop()
			self.dbprint('%d' % int(self.current_heading()))


	def fwd_straightly(self, max_steps=5, max_secs=1, heading=None, power=50):
		self.reset_counters()
		pid = Pid(5., 1., 1.)
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
		finally:
			self.stop()
			self.dbprint('%d' % int(self.current_heading()))

