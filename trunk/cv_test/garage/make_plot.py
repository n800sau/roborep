#!/usr/bin/env python

import glob, os, sys, re, datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

img_path = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')

plt.hold(False)
for dname in glob.glob(os.path.join(img_path, '201*-*-*')):
	tvals = {}
	for fname in glob.glob(os.path.join(dname, '*(n800sau)*_*.jpg')):
#		00D6FB009223(n800sau)_1_20151202020654_2294.jpg
		bname = os.path.basename(fname)
		match = re.match(r'^.+\(n800sau\)_\d_(\d{14})_\d{4}\.jpg$', bname)
		if not match is None:
			ts = datetime.datetime.strptime(match.groups()[0], '%Y%m%d%H%M%S')
			ts.hour
			tvals[ts.hour] = tvals.get(ts.hour, 0) + 1
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


