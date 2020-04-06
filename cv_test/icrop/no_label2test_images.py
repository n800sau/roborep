#!/usr/bin/env python

import os, sys, re, shutil
import redis
from misc import fname2dt

queue_name = ('no_label_' + sys.argv[1]) if len(sys.argv) > 1 else 'no_label'


BASEPATH = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')
DESTPATH = 'data/test_images'
r = redis.Redis('bbspeaker')

for bname in r.lrange(queue_name, 0, -1):
	dt = fname2dt(bname)
	dname = dt.strftime('%Y-%m-%d')
	srcname = os.path.join(BASEPATH, dname, bname)
	destname = os.path.join(DESTPATH, bname)
	shutil.copy(srcname, destname)
	print srcname, '-', destname
