#!/usr/bin/env python3

import os
import dbm
import json
import re
import datetime
from PIL import ExifTags
from PIL import Image

bpath = os.path.expanduser('~/sshfs/asus/root/g750/hdd750/mydvd')

# 2014-06-22_16-06-05_IMg_0149.jpg
# 2014-06-22_16_06_05.jpg
# IMG_20191207_162312.jpg

with dbm.open('imlist.db') as db:

	done = 0
	ts_sorted = {'no_dated': []}
	all_rows = dict(db)
	for fname in all_rows.keys():
		ts = None
		try:
			fname = fname.decode()
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
				img = Image.open(fname)
				exif_data = img._getexif()
				if not exif_data is None:
					exif = {
						ExifTags.TAGS[k]: v for k, v in exif_data.items() if k in ExifTags.TAGS
					}
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
			dt = ts.strftime('%Y-%m-%d')
			if dt not in ts_sorted:
				ts_sorted[dt] = []
			ts_sorted[dt].append(fname[len(bpath):])
			done += 1

	json.dump(ts_sorted, open('ts_sorted.json', 'w'), indent=2, ensure_ascii=False, sort_keys=True)

print('Finished with %d sorted and %d unsorted' % (done, len(ts_sorted['no_dated'])))
