#!/usr/bin/env python

import sys, os
sys.path.append(os.path.expanduser('~/private'))
from credential import cred
import re
from datetime import datetime
from ftplib import FTP
import redis

REDIS_LIST = 'garage_files2label'

#r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')
r = re.compile('([0-9]{14})-\d+(-\d+)?\.(jpg|avi)')

redis = redis.Redis()
ftp_h = FTP('192.168.1.1')
ftp_h.login('writer', cred['writer@192.168.1.1'])
ftp_h.cwd('rus_hard/garage')
#00D6FB009223(n800sau)_1_20160516142921_30928.jpg
pathlist = ftp_h.nlst()
for srcname in pathlist:
	bname = os.path.basename(srcname)
	m = r.match(bname)
	if m and m.groups():
		dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
		bdname = dt.strftime('%Y-%m-%d')
		dname = os.path.join(os.path.dirname(srcname), bdname)
		if os.path.splitext(bname)[-1] == '.avi':
			dname += '_vids'
		else:
			dname += '_pics'
		if dname not in pathlist:
			print 'Current directory', ftp_h.pwd()
			print 'Create directory', dname
			ftp_h.mkd(dname)
			pathlist.append(dname)
		dstname = os.path.join(dname, bname)
		print bname, '->', dstname
		ftp_h.rename(srcname, dstname)
		redis.rpush(REDIS_LIST, dstname)

ftp_h.quit()
