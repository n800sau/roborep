#!/usr/bin/env python

import csv
import matplotlib as mpl
#mpl.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

f = file('q.R.csv')
vf = csv.reader(f)
data_time = []
data_x = []
for row in vf:
    v = dict(zip(row[::2], row[1::2]))
    data_time.append(datetime.fromtimestamp(int(v['secs'])))
    data_x.append(float(v['acc_x']))

#data = np.genfromtxt('q.R.csv', delimiter=',', usecols = (1, 7), names=['x', 'y'])



fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Numbers")
ax1.set_xlabel('time')
ax1.set_ylabel('Z-z-z')

ax1.plot(data_time, data_x, color='r', label='the data')
plt.gcf().autofmt_xdate()

leg = ax1.legend()

plt.show()
#plt.savefig('plot.png', bbox_inches='tight')
