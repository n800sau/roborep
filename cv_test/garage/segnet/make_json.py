#!/usr/bin/env python

import os, sys, json, random

jfname = 'garage.json'
dirname = 'no_car'

if os.path.exists(jfname):
	js = json.load(file(jfname))
else:
	js = []

infiles = list(os.walk(dirname, followlinks=True))
infiles = random.sample(infiles, min(len(infiles), 30))

for root, dirs, files in infiles:
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

json.dump(js, file(jfname, 'w'), indent=2)
