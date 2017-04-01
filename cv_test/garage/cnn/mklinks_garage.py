#!/usr/bin/env python

import sys, glob, random, os, shutil, datetime

label_counter = {}
label_flist = {}

BPATH = '~/sshfs/asus/root/rus_hard/garage'

IPATH = 'input.generated'

if os.path.exists(IPATH):
	shutil.rmtree(IPATH)

bpath = os.path.expanduser(BPATH)

inside_range = [datetime.time(hour=22), datetime.time(hour=6)]
outside_range = [datetime.time(hour=8), datetime.time(hour=17)]

for root, dirs, files in os.walk(bpath, topdown=False, followlinks=True):
	for name in files:
		fname = os.path.join(root, name)
		mtime = os.path.getmtime(fname)
		last_modified_date = datetime.datetime.fromtimestamp(mtime)
		label = None
		weekday = last_modified_date.weekday()
		if weekday >= 0 and weekday < 5:
			if last_modified_date.time() >= inside_range[0] or last_modified_date.time() < inside_range[1]:
				label = 'inside'
			if last_modified_date.time() >= outside_range[0] and last_modified_date.time() < outside_range[1]:
				label = 'outside'
			if label:
				dname = os.path.join(IPATH, label)
				if not os.path.exists(dname):
					os.makedirs(dname)
				dfname = os.path.join(dname, os.path.basename(fname))
				print dfname
				os.symlink(fname, dfname)
