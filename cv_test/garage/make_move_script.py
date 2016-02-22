#!/usr/bin/env python

import sys, os, glob, re, shutil
from datetime import datetime
from subprocess import list2cmdline

img_path = os.path.expanduser('~/sshfs/asus/root/rus_hard/garage')

r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')

cmdfname = 'move.sh'
cmdf = file(cmdfname, 'w')
try:
	jpglist = glob.glob(os.path.join(img_path, '*.jpg'))
	print 'Size:', len(jpglist)
	for f in jpglist:
		bname = os.path.basename(f)
		m = r.match(bname)
		if m:
			dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
			bdname = dt.strftime('%Y-%m-%d')
			dname = os.path.join(os.path.dirname(f), bdname)
			if not os.path.exists(dname):
				os.mkdir(dname)
			fname = os.path.join(dname, bname)
			print >>cmdf, "mv '%s' '%s'" % (bname, os.path.join(bdname, bname))
	#		os.rename(f, fname)
	#		shutil.move(f, fname)
#			print dt.strftime('%Y-%m-%d %H:%M:%S')
	#		print fname
	#	00D6FB009223(n800sau)_1_20151202020654_2294.jpg
finally:
	cmdf.close()
shutil.copy(cmdfname, os.path.join(img_path, cmdfname))
print 'Copied'
