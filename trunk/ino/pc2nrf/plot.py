#!/usr/bin/env python

import csv
import matplotlib as mpl
mpl.use('Agg')
import numpy as np
import matplotlib.pyplot as plt

f = file('q.R.csv')
vf = csv.reader(f)
data_x = []
data_y = []
for row in vf:
    v = dict(zip(row[::2], row[1::2]))
    data_x.append(int(v['secs']))
    data_y.append(float(v['acc_x']))

#data = np.genfromtxt('q.R.csv', delimiter=',', usecols = (1, 7), names=['x', 'y'])



fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Numbers")
ax1.set_xlabel('time')
ax1.set_ylabel('Z-z-z')

ax1.plot(data_x, data_y, color='r', label='the data')

leg = ax1.legend()

#plt.show()
plt.savefig('plot.png', bbox_inches='tight')
