#!/usr/bin/env python

import matplotlib
matplotlib.use('Agg')
import sys, os, csv
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.lines import Line2D

fname = sys.argv[1]

x = []
ysonar = []
ycommand = []
ycompass = []
ygyro = [[], [], []]

# start with the first command
# stop after 10 secs after the mstop command unless another command occurs

active = False
stopsecs = None
i = 0
with open(fname) as csvfile:
	for r in csv.reader(csvfile, delimiter = ','):
		name = r[2].strip()
		if not active:
			if name == 'command':
				active = True
		if active:
			secs = int(r[0])
			x.append(secs + float(r[1]) / 1000000000)
			ycompass.append(r[3] if name == 'compass' else None)
			ysonar.append(r[3] if name == 'state' else None)
			if name == 'gyro':
				ygyro[0].append(r[3])
				ygyro[1].append(r[4])
				ygyro[2].append(r[5])
			else:
				ygyro[0].append(None)
				ygyro[1].append(None)
				ygyro[2].append(None)
			if name == 'command':
				if r[3] == 'mstop':
					ycommand.append(1)
					stopsecs = secs + 10
				elif not stopsecs is None:
					ycommand.append(2)
					stopsecs = None
				else:
					ycommand.append(2)
			else:
				ycommand.append(None)
			i += 1
			if not stopsecs is None:
				if stopsecs < secs:
					break
#		if i > 100:
#			break

fig, axes = plt.subplots(nrows=4, ncols=1, sharex=True, sharey=False)

axes[0].plot(x, ycommand, marker='x', linestyle='', color='cornflowerblue', markersize=5)

axes[1].plot(x, ysonar, color='green', linestyle='-', marker='.', markersize=2)

axes[2].plot(x, ycompass, color='blue', linestyle='-', marker='.', markersize=2)

axes[3].plot(x, ygyro[0], color='green', linestyle='-', marker='.', markersize=2)
axes[3].plot(x, ygyro[1], color='blue', linestyle='-', marker='.', markersize=2)
axes[3].plot(x, ygyro[2], color='orange', linestyle='-', marker='.', markersize=2)

plt.savefig(os.path.expanduser('~/public_html/bag/plot.png'), bbox_inches='tight')

#plt.show()
