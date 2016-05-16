#!/usr/bin/env python

import os, sys, re
from datetime import datetime
from ftplib import FTP

r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')

ftp = FTP('192.168.1.1')
ftp.login('writer', 'pfgbcm')
ftp.cwd('rus_hard/garage')
#00D6FB009223(n800sau)_1_20160516142921_30928.jpg
pathlist = ftp.nlst()
for srcname in pathlist:
	bname = os.path.basename(srcname)
	m = r.match(bname)
	if m:
		dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
		bdname = dt.strftime('%Y-%m-%d')
		dname = os.path.join(os.path.dirname(srcname), bdname)
		if dname not in pathlist:
			ftp.mkd(dname)
		dstname = os.path.join(dname, bname)
		print bname, '->', dstname
		ftp.rename(srcname, dstname)

ftp.quit()
