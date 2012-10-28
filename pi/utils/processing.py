#!/usr/bin/env python

import numpy as np
import pandas as pd
import csv
from datetime import datetime

def to_datetime(sval):
	return datetime.strptime(sval, '%Y.%m.%d %H:%M:%S')

def b1v(sval):
	return float(sval)/1024*4.096 * (367. / 99)

def b2v(sval):
	return float(sval)/1024*4.096 * (367. / 99)

def sw1v(sval):
	return float(sval)/1024*4.096 * (365. / 97)

def sw2v(sval):
	return float(sval)/1024*4.096 * (3638. / 978)

reader = csv.reader(file('data.log'))
converters = [to_datetime, b1v, b2v, sw1v, sw2v]
#reader = csv.reader(file('data.log'))
#data = np.array([[conv(col) for col, conv in zip(row, converters)] for row in reader])
cdata = []
for row in reader:
	rdata = []
	try:
		for col, conv in zip(row, converters):
			rdata.append(conv(col))
		cdata.append(rdata)
	except ValueError, e:
		print e
data = np.array(cdata)

print data.shape, data[:, 1:].shape
ndx = data[:,0]
print 'index len', len(ndx)

df = pd.DataFrame(data[:, 1:], index=ndx, columns=['b1', 'b2', 'sw1', 'sw2'])
ts = df['b1']

def rmean(ts):
    return ts.mean()

rs = df.resample('1min', how=rmean)
#print rs

pieces = {'bc': rs['b1']-rs['b2'], 'swc': rs['sw1']-rs['sw2']}
ts = pd.concat(pieces, axis=1)

ts = ts.cumsum()

import matplotlib
matplotlib.use('Agg')
ts.plot().figure.savefig("plot.png")



