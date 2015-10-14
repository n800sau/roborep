import sys, os, time, datetime

def dbprint(text):
	print >>sys.__stderr__, '[%s]:%s' % (datetime.datetime.fromtimestamp(time.time()).strftime('%d/%m/%Y %H:%M:%S.%f'), text)

# angle difference in degrees
def angle_diff(a1, a2):
	return ((a1 - a2) + 180) % 360 - 180

HTML_PATH = os.path.expanduser('~/public_html')

def html_path(fname):
	return os.path.join(HTML_PATH, fname)
