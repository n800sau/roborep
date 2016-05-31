#!/usr/bin/env python

import os, sys, glob
from subprocess import list2cmdline
from pipes import quote
from imutils import paths

SUBJ_PATH = 'data/test_images'
LABEL_PATH = 'data/labels'

bnamelist = [os.path.basename(fname) for fname in paths.list_images(LABEL_PATH)]

fname_list = glob.glob(os.path.join(SUBJ_PATH, '*'))

cmdlist = []
for fname in fname_list:
	bname = os.path.basename(fname)
	if bname not in bnamelist:
		cmdlist.append(list2cmdline(['rm',  quote(fname)]))

print '\n'.join(cmdlist)

print '# %s to remove, %d to remain, %d in labels' % (len(cmdlist), len(fname_list)-len(cmdlist), len(bnamelist))
