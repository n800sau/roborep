#!/usr/bin/env python

import sys, os, csv
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.lines import Line2D

fname = sys.argv[1]

tops = []

x = []
y = []

marker_style = dict(linestyle=':', color='cornflowerblue', markersize=10)

i = 0
with open(fname) as csvfile:
	for r in csv.reader(csvfile, delimiter = ','):
		x.append(int(r[0]) + float(r[1]) / 1000000000)
		if r[2] not in tops:
			tops.append(r[2])
		y.append(tops.index(r[2]))
		i += 1
		if i > 100:
			break

fig, ax = plt.subplots()
ax.plot(x, y, marker='*', **marker_style)

#savefig('foo.png', bbox_inches='tight')

plt.show()
