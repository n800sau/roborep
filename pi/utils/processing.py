#!/usr/bin/env python

import numpy as np
import pandas as pd
import csv
from datetime import datetime

def to_datetime(sval):
    return datetime.strptime(sval, '%Y.%m.%d %H:%M:%S')

reader = csv.reader(file('data.log'))
converters = [to_datetime, float, float, float, float, float, float]
data = np.array([[conv(col) for col, conv in zip(row, converters)] for row in reader])
ndx = data[:,0]

df = pd.DataFrame(data[:, 1:], index=ndx, columns=['bc', 'b1', 'b2', 'swc', 'sw1', 'sw2'])
ts = df['bc']

def rmean(ts):
    return ts.mean()

rs = df.resample('10min', how=rmean)
print rs