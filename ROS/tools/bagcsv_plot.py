#!/usr/bin/env python

import matplotlib
matplotlib.use('Agg')
import sys, os, csv
import datetime as dt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.lines import Line2D

fname = sys.argv[1]

x = []
ysonar = []
ycommand = []
xcompass = []
ycompass = []
xgyro = []
ygyro = [[], [], []]
xacc = []
yacc = [[], [], []]
ximu = []
yimu = []

# start with the first command
# stop after 10 secs after the mstop command unless another command occurs

cmd_val = 0
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
			if name == 'compass':
				xcompass.append(x[-1])
				ycompass.append(r[3])
			if name == 'imu':
				ximu.append(x[-1])
				yimu.append(r[3])
			ysonar.append(r[3] if name == 'state' else None)
			if name == 'gyro':
				xgyro.append(x[-1])
				ygyro[0].append(r[3])
				ygyro[1].append(r[4])
				ygyro[2].append(r[5])
			if name == 'acc':
				xacc.append(x[-1])
				yacc[0].append(r[3])
				yacc[1].append(r[4])
				yacc[2].append(r[5])
			if name == 'command':
				if r[3] == 'mstop':
					cmd_val = 0
					stopsecs = secs + 10
				elif not stopsecs is None:
					cmd_val = 1
					stopsecs = None
				else:
					cmd_val = 1
			ycommand.append(cmd_val)
			i += 1
			if not stopsecs is None:
				if stopsecs < secs:
					break
#		if i > 100:
#			break

fig, axes = plt.subplots(nrows=5, ncols=1, sharex=True, sharey=False)

#axes[0].bar(x, ycommand, edgecolor='cornflowerblue', color='gray')
axes[0].plot(x, ycommand, marker='x', linestyle='', color='cornflowerblue', markersize=5)

legends = []

axes[1].plot(x, ysonar, color='green', linestyle='-', marker='.', markersize=2, label='Dist')
legends.append(axes[1].legend(loc='upper right', shadow=True))

if len(ycompass) > 3:
	y = np.interp(x, xcompass, ycompass)
	axes[2].plot(x, y, color='blue', linestyle='-', marker='.', markersize=2, label='Head')

if len(yimu) > 3:
	y = np.interp(x, ximu, yimu)
	axes[2].plot(x, y, color='green', linestyle='-', marker='.', markersize=2, label='Angl')

legends.append(axes[2].legend(loc='upper right', shadow=True))

for i,l,c in ((0, 'Gx', 'green'), (1, 'Gy', 'blue'), (2, 'Gz', 'orange')):
	if len(ygyro[i]) > 3:
		y = np.interp(x, xgyro, ygyro[i])
		axes[3].plot(x, y, color=c, linestyle='-', marker='.', markersize=2, label=l)
legends.append(axes[3].legend(loc='upper right', shadow=True))

for i,l,c in ((0, 'Ax', 'green'), (1, 'Ay', 'blue'), (2, 'Az', 'orange')):
	if len(yacc[i]) > 3:
		y = np.interp(x, xacc, yacc[i])
		axes[4].plot(x, y, color=c, linestyle='-', marker='.', markersize=2, label=l)
legends.append(axes[4].legend(loc='upper right', shadow=True))

for legend in legends:

	# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
	frame = legend.get_frame()
	frame.set_facecolor('0.90')

	# Set the fontsize
	for label in legend.get_texts():
		label.set_fontsize('small')

	for label in legend.get_lines():
		label.set_linewidth(1.5)  # the legend line width

plt.savefig(os.path.expanduser('~/public_html/bag/plot.png'), bbox_inches='tight')

#plt.show()
