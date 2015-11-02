#!/usr/bin/env python

import json, os

data = json.load(file('data.json'))
df = file('data', 'w')
print >>df, len(data), len(data[0]['input']), len(data[0]['output'])
for d in data:
	print >>df, '%g' % d['input']['pwr'],
	print >>df, '%g' % d['output']['adiff'],
	print >>df
df.close()

dfname = 'fann/pwr/data'
if os.path.exists(dfname):
	os.unlink(dfname)
os.rename('data', dfname)
