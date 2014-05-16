#!/usr/bin/env python

import matplotlib as mpl
mpl.use('Agg')
import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('q.S.csv', delimiter=',', usecols = (1, 3), names=['x', 'y'])

fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title("Numbers")
ax1.set_xlabel('time')
ax1.set_ylabel('Z-z-z')

ax1.plot(data['x'], data['y'], color='r', label='the data')

leg = ax1.legend()

#plt.show()
plt.savefig('plot.png', bbox_inches='tight')
