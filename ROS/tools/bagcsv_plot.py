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
xstate = []
ysonar = []
ylpwr = []
yrpwr = []
ylcurrent = []
yrcurrent = []
ylcount = []
yrcount = []
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
			if name == 'state':
				xstate.append(x[-1])
				ysonar.append(r[3])
				ylpwr.append(r[4])
				yrpwr.append(r[5])
				ylcurrent.append(r[6])
				yrcurrent.append(r[7])
				ylcount.append(r[8])
				yrcount.append(r[9])
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

fig, axes = plt.subplots(nrows=8, ncols=1, sharex=True, sharey=False)
fig.set_size_inches(6, 8)

def do_plot(ax, xlist, ylist, color, label):
	if len(ylist) > 3:
		y = np.interp(x, xlist, ylist)
	ax.plot(x, y, color=color, linestyle='-', marker='.', markersize=2, label=label)

#axes[0].bar(x, ycommand, edgecolor='cornflowerblue', color='gray')

axes[0].plot(x, ycommand, marker='x', linestyle='', color='cornflowerblue', markersize=5)


legends = []

do_plot(axes[1], xstate, ysonar, 'green', 'Dist')
legends.append(axes[1].legend(loc='upper right', shadow=True))

do_plot(axes[2], xcompass, ycompass, 'blue', 'Head')
do_plot(axes[2], ximu, yimu, 'green', 'Angl')
legends.append(axes[2].legend(loc='upper right', shadow=True))

for i,l,c in ((0, 'Gx', 'green'), (1, 'Gy', 'blue'), (2, 'Gz', 'orange')):
	do_plot(axes[3], xgyro, ygyro[i], c, l)
legends.append(axes[3].legend(loc='upper right', shadow=True))

for i,l,c in ((0, 'Ax', 'green'), (1, 'Ay', 'blue'), (2, 'Az', 'orange')):
	do_plot(axes[4], xacc, yacc[i], c, l)
legends.append(axes[4].legend(loc='upper right', shadow=True))

do_plot(axes[5], xstate, ylpwr, 'blue', 'lPwr')
do_plot(axes[5], xstate, yrpwr, 'green', 'rPwr')
legends.append(axes[5].legend(loc='upper right', shadow=True))

do_plot(axes[6], xstate, ylcurrent, 'blue', 'lCurr')
do_plot(axes[6], xstate, yrcurrent, 'green', 'rCurr')
legends.append(axes[6].legend(loc='upper right', shadow=True))

do_plot(axes[7], xstate, ylcount, 'blue', 'lCnt')
do_plot(axes[7], xstate, yrcount, 'green', 'rCnt')
legends.append(axes[7].legend(loc='upper right', shadow=True))

for legend in legends:

	# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
	frame = legend.get_frame()
	frame.set_facecolor('0.90')

	# Set the fontsize
	for label in legend.get_texts():
		label.set_fontsize('small')

	for label in legend.get_lines():
		label.set_linewidth(1.5)  # the legend line width

plt.savefig(os.path.expanduser('~/public_html/bag/plot.png'), bbox_inches='tight', dpi = 100)

#plt.show()
