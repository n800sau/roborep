#!/usr/bin/env python3

import os
import dbm
import json
from collections import OrderedDict

with dbm.open('imlist.db') as db:

	h_files = {}
	all_rows = OrderedDict(db)
	for fname,h in all_rows.items():
		if h not in h_files:
			h_files[h] = []
		h_files[h].append(fname.decode())

	dirset = set()
	dups = []
	for h,fnames in h_files.items():
		if len(fnames) > 1:
			dups.append(fnames)
			for fname in fnames:
				dirset.add(os.path.dirname(fname))
	dirset = list(dirset)
	dirset.sort()

	json.dump(dups, open('dupfiles.json', 'w'), indent=2)
	json.dump(dirset, open('dupdirs.json', 'w'), indent=2)
	print('found dups: %d, dirs: %d' % (len(dups), len(dirset)))
