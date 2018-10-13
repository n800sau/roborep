#!/usr/bin/env python

import os, sys, time, traceback, re
sys.path.append(os.path.expanduser('~n800s/private'))
import redis
from ftplib import FTP, all_errors
from credential import cred

R_QUEUE = 'q.motion_captured'

reg = re.compile('([0-9]{14})-\d+(-\d+)?\.(jpg|avi)')

r = redis.Redis()

pathlist = None
while True:
	try:
		if r.llen(R_QUEUE) > 0:
			ftp_h = FTP('192.168.1.1')
			try:
				ftp_h.login('writer', cred['writer@192.168.1.1'])
				ftp_h.cwd('rus_hard/garage')
				if pathlist is None:
					pathlist = ftp_h.nlst()
				while True:
					fname = r.rpop(R_QUEUE)
					if not fname:
						break
					if os.path.exists(fname):
						print 'Sending', fname, '...'
						bname = os.path.basename(fname)
						m = reg.match(bname)
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
							f = open(fname, 'rb')
							try:
								ftp_h.storbinary('STOR ' + os.path.join(dname, bname), f)
								f.close()
								os.unlink(fname)
							except:
								r.lpush(R_QUEUE, fname)
								raise
			finally:
				ftp_h.quit()
		else:
			time.sleep(1)
	except all_errors:
		traceback.print_exc()
		time.sleep(10)
		continue
