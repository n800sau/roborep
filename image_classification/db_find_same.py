#!/usr/bin/env python3

import os
import shelve
import json
from collections import OrderedDict

with shelve.open('imlist.shelve') as db:

	h_files = {}
	all_rows = OrderedDict(db)
	for fname,data in all_rows.items():
		h = data['md5sum']
		if h not in h_files:
			h_files[h] = []
		h_files[h].append(fname)

	dircounts = {}
	dups = []
	for h,fnames in h_files.items():
		if len(fnames) > 1:
			dups.append(fnames)
			for fname in fnames:
				dname = os.path.dirname(fname)
				if dname not in dircounts:
					dircounts[dname] = 0
				dircounts[dname] += 1

	json.dump(dups, open('dupfiles.json', 'w'), indent=2)
	json.dump(dircounts, open('dupdirs.json', 'w'), indent=2)
	print('found dups: %d, dirs: %d' % (len(dups), len(dircounts)))
