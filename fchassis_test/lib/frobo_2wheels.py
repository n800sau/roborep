import math
from frobo_common import frobo_common

COUNT_PER_REV = 20.0
WHEEL_DIAMETER = 0.065
BASELINE = 0.14

ENC_STEP = WHEEL_DIAMETER * math.pi / COUNT_PER_REV

class frobo_2wheels(frobo_common):

	def turn(self, *args, **kwds):
		return self.turn_in_ticks(*args, **kwds)

	def m2steps(self, m):
		return int(m / ENC_STEP)

	def steps2m(self, steps):
		return steps * ENC_STEP
