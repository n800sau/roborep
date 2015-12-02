from frobo_common import frobo_common

class frobo_track(frobo_common):

	def turn(self, *args, **kwds):
		return self.simple_turn(*args, **kwds)
