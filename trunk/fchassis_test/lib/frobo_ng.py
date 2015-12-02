from socket import gethostname
from frobo_track import frobo_track
from frobo_2wheels import frobo_2wheels

def frobo_ng(*args, **kwds):
	rs = None
	hn = gethostname()
	if hn == 'orange':
		rs = frobo_track(*args, **kwds)
	elif hn == 'hubee':
		rs = frobo_2wheels(*args, **kwds)
	return rs
