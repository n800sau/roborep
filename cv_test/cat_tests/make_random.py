#!/usr/bin/env python

import os, sys, imghdr, time, shutil, random

MAX_N = 5000

SRCDIR=os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
OUTDIR=os.path.expanduser('~/work/garage/random')

if os.path.exists(OUTDIR):
	shutil.rmtree(OUTDIR)

os.makedirs(OUTDIR)

CACHE_FNAME = 'all_files.lst'

# make list of all
if os.path.exists(CACHE_FNAME):
	flist = [v.strip() for v in file(CACHE_FNAME).read().split('\n') if v.strip()]
else:
	cache_file = file(CACHE_FNAME, 'w')
	try:
		flist = []
		for root, dirs, files in os.walk(SRCDIR, followlinks=True):
			for fname in files:
				fname = os.path.join(os.path.join(root, fname))
				try:
					itype = imghdr.what(fname)
					if itype :
						flist.append(fname)
						cache_file.write(fname + '\n')
				except Exception, e:
					print e
	finally:
		cache_file.close()


flist = random.sample(list(flist), MAX_N)

i = 0
for fname in flist:
#	os.symlink(fname, os.path.join(OUTDIR, os.path.basename(fname)))
	shutil.copyfile(fname, os.path.join(OUTDIR, os.path.basename(fname)))
	i += 1
	print '%3d: %s' % (i, fname)

print 'Finished'
