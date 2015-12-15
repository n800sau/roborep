import time
from utils import angle_diff, html_path, html_data_path
from frobo_common import frobo_common

#encoders 0.25 mm per unit?
#ENC_STEP = 0.003 # test result back?
#ENC_STEP = 0.0015 # test result fwd
ENC_STEP = 0.003

class frobo_track(frobo_common):

	def turn(self, *args, **kwds):
		return self.simple_turn(*args, **kwds)

	def move_straight(self, fwd=True, max_steps=5, max_secs=1, heading=None, power=50):
		self.cmd_reset_counters()
		counted_offset = 0
		if heading is None:
			heading = self.heading()
		elif abs(angle_diff(heading, self.heading())) > 5:
			self.turn_in_ticks(heading)
		offset = 0
		try:
			t = time.time()
			while (t + max_secs) > time.time():
				self.update_state()
				hdiff = angle_diff(self.heading(), heading)
				if abs(hdiff) > 20:
					self.dbprint("STOP and CORRECT")
					tt = time.time()
					counted = self.steps_counted()
					self.turn_in_ticks(heading, err=5)
					counted_offset = counted - self.steps_counted()
					self.dbprint("COUNTER OFFSET=%d" % counted_offset)
					hdiff = angle_diff(self.heading(), heading)
					max_secs += time.time() - tt
				if hdiff > 1:
					offset = 20 if fwd else -20
				elif hdiff < -1:
					offset = -20 if fwd else 20
				else:
					offset = 0
				lpwr = power - offset
				rpwr = power + offset
				self.dbprint('^%d hdiff:%g off:%d pw:%d<>%d cnt:%d<>%d' % (int(self.heading()), hdiff, offset, lpwr, rpwr, self.state['lcount'], self.state['rcount']))
				if fwd and self.state['sonar'] >= 0 and self.state['sonar'] < self.MIN_DISTANCE:
					self.hit_warn = self.state['sonar']
					self.dbprint('STOP distance=%s' % self.state['sonar'])
					break
				steps = self.steps_counted() + counted_offset
				if steps > max_steps:
					self.dbprint('Max steps reached (%d > %d)' % (steps, max_steps))
					hdiff = angle_diff(self.heading(), heading)
					if abs(hdiff) > 3:
						self.dbprint("FINAL CORRECTION")
						self.turn_in_ticks(heading, err=5)
					break
				self.cmd_mboth(lpwr, fwd, rpwr, fwd)
				time.sleep(self.STEP_TIME)
				self.update_state()
		finally:
			self.cmd_mstop()

	def m2steps(self, m):
		return int(m / ENC_STEP)

