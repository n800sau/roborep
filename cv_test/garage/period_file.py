#!/usr/bin/env python

import glob, os, sys, json
from misc import fname2dt, fname2dt_exc

IMG_PATH = os.path.expanduser('output/images/predict')

for dname in glob.glob(os.path.join(IMG_PATH, '2016-*-*')):
	if os.path.isdir(dname):
		data = []
		for lname in glob.glob(os.path.join(dname, '*')):
			if os.path.isdir(lname):
				label = os.path.basename(lname)
				for fname in glob.glob(os.path.join(lname, '*(n800sau)*_*.jpg')):
					try:
						ts = fname2dt(os.path.basename(fname))
						data.append((ts.strftime('%H-%M-%S'), label))
					except fname2dt_exc:
						print '%s skipped' % os.path.basename(fname)
		data.sort()
		ofname = os.path.join(dname, 'labels.json')
		json.dump(data, file(ofname, 'w'), indent=2)
		pdata = []
		pleft = None
		label = None
		for p1,p2 in zip(data, data[1:] + [('24-59-59', 'end')]):
			if label is None:
				label = p1[1]
				pleft = p1[0]
			if p1[1] != p2[1]:
				pdata.append((label, pleft, p2[0]))
				label = p2[1]
				pleft = p2[0]
		ofname = os.path.join(dname, 'periods.json')
		json.dump(pdata, file(ofname, 'w'), indent=2)

