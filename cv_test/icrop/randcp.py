#!/usr/bin/env python

import sys, os, random, glob, shutil

flist = glob.glob(os.path.join(sys.argv[1], '*'))

rflist = random.sample(flist, 50)

for fname in rflist:
	shutil.copy(fname, sys.argv[2])

