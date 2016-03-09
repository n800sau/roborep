#!/usr/bin/env python

import glob, os, sys, re, datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from misc import fname2dt, fname2dt_exc

img_path = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')

plt.hold(False)
for dname in glob.glob(os.path.join(img_path, '2016-03-*')):
	tvals = {}
	for fname in glob.glob(os.path.join(dname, '*(n800sau)*_*.jpg')):
		try:
			ts = fname2dt(fname)
			ts.hour
			tvals[ts.hour] = tvals.get(ts.hour, 0) + 1
		except fname2dt_exc:
			pass
	hours = range(1, 25)
	hist = []
	for h in hours:
		hist.append(tvals.get(h, 0))
	fig, ax = plt.subplots()
	width = 0.35
	plt.bar(hours, hist, width, color='b', )
	plt.xlabel('Hours')
	plt.ylabel('Scores')
	plt.title('Scores per hour')
	plt.xticks(np.array(hours) + width, hours)
	plt.tight_layout()
	dfname = os.path.join(dname, 'plot.png')
	plt.savefig(dfname)
	plt.cla()


