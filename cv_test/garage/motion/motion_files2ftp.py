#!/usr/bin/env python

import os, sys, time, traceback, re, glob
sys.path.append(os.path.expanduser('~n800s/private'))
from datetime import datetime
from ftplib import FTP, all_errors
from credential import cred


SRCDPATH = '/var/lib/motion'

reg = re.compile('([0-9]{14})-\d+(-\d+)?\.(jpg|avi)')
BASE_DNAME = 'rus_hard/garage'

pathlist = None
while True:
	try:
		flist =[fname for fname in glob.glob(os.path.join(SRCDPATH, '*.*')) if os.path.splitext(fname)[1] in ('.avi', '.jpg')]
		if len(flist) > 0:
			ftp_h = FTP('192.168.1.1')
			try:
				ftp_h.login('writer', cred['writer@192.168.1.1'])
				ftp_h.cwd(BASE_DNAME)
				if pathlist is None:
					pathlist = [os.path.basename(dname) for dname in ftp_h.nlst() if re.match(r'^\d+-\d+-\d+_((pics)|(vids))$', os.path.basename(dname))]
#					print pathlist
				for fname in flist:
					if os.path.isfile(fname):
						bname = os.path.basename(fname)
						m = reg.match(bname)
						if m and m.groups():
							dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
							dname = dt.strftime('%Y-%m-%d')
							if os.path.splitext(bname)[-1] == '.avi':
								dname += '_vids'
							else:
								dname += '_pics'
							if dname not in pathlist:
								print 'Current directory', ftp_h.pwd()
								print 'Create directory', dname
								pathlist.append(dname)
								ftp_h.mkd(dname)
								# to test successfully created directory
								ftp_h.cwd(dname)
								ftp_h.cwd('..')
							dfname = os.path.join(dname, bname)
							print 'Sending', fname, ' to ', dfname, '...'
							f = open(fname, 'rb')
							ftp_h.storbinary('STOR ' + dfname, f)
							f.close()
							print fname, 'sent'
							os.unlink(fname)
			finally:
				ftp_h.quit()
		else:
			time.sleep(1)
	except all_errors:
		traceback.print_exc()
		time.sleep(10)
		continue
