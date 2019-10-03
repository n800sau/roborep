#!/usr/bin/env python

import os, sys, json

jfname = 'trucks.json'
dirname = 'images'

if os.path.exists(jfname):
	js = json.load(file(jfname))
else:
	js = []

for root, dirs, files in os.walk(dirname, followlinks=True):
	for name in files:
		fname = os.path.join(root, name)
		found = False
		for iobj in js:
			if iobj['filename'] == fname:
				found = True
				break
		if not found:
			js.append({
				'filename': fname,
				'class': 'image',
			})

json.dump(js, file(jfname, 'w'), indent=4)
