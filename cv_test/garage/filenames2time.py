#!/usr/bin/env python

import sys, os, glob, re, shutil, time, json
from datetime import datetime, timedelta
from subprocess import list2cmdline
import numpy
import pylab as P
import matplotlib.dates as dates

time_mark = int(time.time())

img_path = os.path.expanduser('~/sshfs/asus/root/sdb1/garage')
out_path = os.path.join(os.path.dirname(__file__), 'output')

r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')

dirlist = glob.glob(os.path.join(img_path, '2015*'))

hourFormatter = P.DateFormatter('%H')

fnames = []

for dname in dirlist:

	bdname = os.path.basename(dname)
	jpglist = glob.glob(os.path.join(dname, '*.jpg'))
	print '%s size:%d' % (bdname, len(jpglist))
	hist = []
	for f in jpglist:
		bname = os.path.basename(f)
		m = r.match(bname)
		if m:
			dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
#			print dt.strftime('%Y %H:%M:%S')
#			print time.mktime(dt.timetuple())
#			print datetime.fromordinal(int(time.mktime(dt.timetuple())))
#			hist.append(int(time.mktime(dt.timetuple())))
			hist.append(dates.date2num(dt))

	bins = []
	for h in range(24):
		bins.append(dates.date2num(dt.replace(hour=h, minute=0, second=0)))
#	print dt, (dt + timedelta(days=1)).replace(hour=0, minute=0, second=0)
	bins.append(dates.date2num((dt + timedelta(days=1)).replace(hour=0, minute=0, second=0)))
#	print bins

	fig, ax = P.subplots()
	ax.xaxis_date()
	n, bins, patches = ax.hist(hist, bins, normed=0, histtype='bar', rwidth=0.8)
#	print n
#	ax.set_xticks(bins)
	ax.xaxis.set_major_formatter(hourFormatter)
	ax.xaxis.set_major_locator(P.HourLocator(P.arange(0, 25)))
	fn = os.path.join('images', 'hist_%s.png' % bdname)
	fnames.append(fn)
	P.savefig(os.path.join(out_path, fn), bbox_inches='tight', format='png')
#	ax.set_xlim(dt.replace(hour=0, minute=0, second=0), dt.replace(hour=23, minute=59, second=59))
#print 'here'
#P.show()
#break

json.dump({
	'files': fnames,
	'title': os.path.splitext(os.path.basename(__file__))[0],
	'time_mark': time_mark,
}, file(os.path.join(out_path, 'frames.json'), 'w'), indent=2)
