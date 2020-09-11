#!/usr/bin/env python3

import os
import shelve
import json
import re
import datetime
from PIL import ExifTags
from PIL import Image

bpath = os.path.expanduser('~/sshfs/asus/root/g750/hdd750/mydvd')

# 2014-06-22_16-06-05_IMg_0149.jpg
# 2014-06-22_16_06_05.jpg
# IMG_20191207_162312.jpg

with shelve.open('imlist.shelve') as db:

	done = 0
	ts_sorted = {'no_dated': []}
	all_rows = dict(db)
	for fname,exif in all_rows.items():
		ts = None
		try:
			bname = os.path.basename(fname)
#			print(bname)
			for ex,df in (
						(r'\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}', '%Y-%m-%d_%H-%M-%S'),
						(r'\d{4}-\d{2}-\d{2}_\d{2}_\d{2}_\d{2}', '%Y-%m-%d_%H_%M_%S'),
						(r'IMG_\d{4}\d{2}\d{2}_', 'IMG_%Y%m%d_'),
					):
				match = re.search(ex, bname)
				if match:
					ts = datetime.datetime.strptime(match.group(), df)
					break
		except ValueError:
			pass
		if ts is None:
			try:
				if 'DateTime' in exif:
					print('exif: %s' % exif['DateTime'])
					try:
						ts = datetime.datetime.strptime(exif['DateTime'], '%Y:%m:%d %H:%M:%S')
					except ValueError:
						pass
			except Exception as e:
				print(e)
			if ts is None:
				ts_sorted['no_dated'].append(fname[len(bpath):])
		if not ts is None:
			y = ts.strftime('%Y')
			if y not in ts_sorted:
				ts_sorted[y] = {}
			m = ts.strftime('%Y-%m')
			if m not in ts_sorted[y]:
				ts_sorted[y][m] = {}
			d = ts.strftime('%Y-%m-%d')
			if d not in ts_sorted[y][m]:
				ts_sorted[y][m][d] = []
			ts_sorted[y][m][d].append(fname[len(bpath):])
			done += 1

	json.dump(ts_sorted, open('ts_sorted.json', 'w'), indent=2, ensure_ascii=False, sort_keys=True)

print('Finished with %d sorted and %d unsorted' % (done, len(ts_sorted['no_dated'])))
