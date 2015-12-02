from frobo_common import frobo_common

class frobo_2wheels(frobo_common):

	def turn(self, *args, **kwds):
		return self.turn_in_ticks(*args, **kwds)
