#!/usr/bin/env python

import glob, random, os, shutil

srcdir = 'dogs_and_cats'
dstdir = 'data'

classes = [os.path.basename(d) for d in glob.glob(os.path.join(srcdir, '*')) if os.path.isdir(d)]

train_part = 0.5

shutil.rmtree(dstdir)

fnames = {}

for cl in classes:
	fnames[cl] = glob.glob(os.path.join(srcdir, cl, '*.jpg'))
	random.shuffle(fnames[cl])
	for v,part in (('train', (0, 0.5, 1)), ('validation', (0, 0.5, -1))):
		dp = os.path.join(dstdir, v, cl)
		if not os.path.exists(dp):
			os.makedirs(dp)
		i = 1
		cnt = len(fnames[cl])
		for fn in fnames[cl][::part[2]][int(cnt * part[0]):int(cnt * part[1])]:
			os.symlink(os.path.abspath(fn), os.path.join(dp, '%s%03d.jpg' % (cl, i)))
			i += 1
