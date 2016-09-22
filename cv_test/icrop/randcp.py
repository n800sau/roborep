#!/usr/bin/env python

import sys, os, random, glob, shutil


BASEPATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
#BASEPATH = os.path.expanduser('~/work/opencv/PyImageSearch/Caltech101/256_ObjectCategories/031.car-tire')

dlist = glob.glob(os.path.join(BASEPATH, '*'))
n = len(dlist)

flist = []
i = 0
for dname in dlist:
	if os.path.isdir(dname):
		print >>sys.stdout, '%d of %d\r' % (i, n),
		sys.stdout.flush()
		flist += glob.glob(os.path.join(dname, '*.jpg'))
		i += 1
	else:
		if os.path.splitext(dname)[-1] == '.jpg':
			flist.append(dname)

print '\nFound %d' % len(flist)

rflist = random.sample(flist, 50)

for fname in rflist:
	shutil.copy(fname, os.path.join('data', 'images'))

