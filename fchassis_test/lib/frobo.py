import time
from hmc5883l import hmc5883l
from fchassis import fchassis
from utils import angle_diff
from pids import Pid

STEP_TIME = 0.1

class frobo(fchassis):

	def __init__(self, *args, **kwds):
		self.dots = []
		self.compass = hmc5883l(gauss = 4.7, declination = (12, 34))
		super(frobo, self).__init__(*args, **kwds)
		self.current_heading()

	def current_heading(self):
		self.last_heading = self.compass.heading()
		return self.last_heading

	def heading_diff(self, azim, err=10):
		heading = self.current_heading()
		diff = angle_diff(azim, heading)
		return diff if abs(diff) > err else 0

	def count_change(self, pin, step, t, dt):
		super(frobo, self).count_change(pin, step, t, dt)
		dot = {
			'heading': self.last_heading,
			'pin': pin,
			'step': step,
			'dt': dt,
			't': t,
		}
		self.dots.append(dot)

	def turn(self, azim, err=10):
		self.dbprint('%d' % int(self.current_heading()))
		try:
			diff = self.heading_diff(azim, err=err)
			if diff:
				if diff > 0:
					self.left_move(True, 50)
					self.right_move(False, 50)
				else:
					self.left_move(False, 50)
					self.right_move(True, 50)
				# maximum 1000 steps ~ 100 sec
				for i in range(1000):
					diff = self.heading_diff(azim, err=err)
					self.dbprint("diff=%d" % diff)
					if not diff:
						break
					self.db_state()
					time.sleep(STEP_TIME)
		finally:
			self.stop()
			self.dbprint('%d' % int(self.current_heading()))


	def fwd_straightly(self, max_steps=5, max_secs=1, heading=None, power=70):
		pid = Pid(2., .1, .1)
		pid.range(-power, power)
		if heading is None:
			heading = self.current_heading()
		pid.set(heading)
		offset = 0
		try:
			for i in range(int(max_secs/STEP_TIME)):
				lpwr = power + offset
				rpwr = power -offset
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

