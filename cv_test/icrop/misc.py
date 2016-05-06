import os, re, datetime

class fname2dt_exc(Exception):
	pass

# 00D6FB009223(n800sau)_1_20151202020654_2294.jpg
# 00D6FB009223(n800sau)_1_20160218110531_28316.jpg
def fname2dt(fname):
	rs = None
	bname = os.path.basename(fname)
	rexp = r'^.+\(n800sau\)_\d_(\d{14})_\d+\.jpg$'
	match = re.match(rexp, bname)
	if match is None:
		raise fname2dt_exc('File name does not match agains %s' % rexp)
	return datetime.datetime.strptime(match.groups()[0], '%Y%m%d%H%M%S')

