#!/usr/bin/env python3

from __future__ import print_function
import os, sys, time, traceback, re, glob
sys.path.append(os.path.expanduser('~n800s/private'))
from datetime import datetime
from ftplib import FTP, all_errors
from credential import cred

# no test for existing files on FTP

SRCDPATH = '/var/lib/motion'

reg = re.compile('([0-9]{14})-\d+(-\d+)?\.(jpg|avi|mp4)')

#BASE_DNAME = 'g750/garage'
BASE_DNAME = os.path.join('g750', sys.argv[1])

def dbprint(text, nl=True):
	print(time.strftime('[%Y-%m-%d %H:%M:%S]'), text, end='')
	if nl:
		print()

pathlist = None
attempt = 0
while True:
	try:
		if attempt > 0:
			print('.', end='')
		else:
			dbprint('Collecting files ', nl=False)
		now = time.time()
		flist =[fname for fname in glob.glob(os.path.join(SRCDPATH, '*.*')) if os.path.splitext(fname)[1] in ('.avi', '.jpg', '.mp4') and os.path.getmtime(fname) < (now - 60)]
		attempt += 1
		if len(flist) > 0:
			dbprint('\n%d found' % len(flist))
			ftp_h = FTP()
			ftp_h.connect('192.168.1.1', 21, 5)
			try:
				ftp_h.login('garage', cred['garage@192.168.1.1'])
				ftp_h.cwd(BASE_DNAME)
				# to start from the most fresh files
				for fname in sorted(flist, reverse=True):
					if os.path.isfile(fname):
						bname = os.path.basename(fname)
						m = reg.match(bname)
						if m and m.groups():
							dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
							dname = dt.strftime('%Y-%m-%d')
							if os.path.splitext(bname)[-1] in ('.avi', '.mp4'):
								dname += '_vids'
							else:
								dname += '_pics'
							dbprint('Current directory: %s' % ftp_h.pwd())
							try:
								ftp_h.mkd(dname)
								dbprint('Created directory %s' % dname)
							except:
								pass
							# to test successfully created or existing directory
							ftp_h.cwd(dname)
							ftp_h.cwd('..')
							dfname = os.path.join(dname, bname)
							print('Sending %s (%d kb) to %s ...' % (fname, os.path.getsize(fname)//1000, dfname))
							f = open(fname, 'rb')
							ftp_h.storbinary('STOR ' + dfname, f)
							f.close()
							print('%s sent' % fname)
							os.unlink(fname)
			finally:
				ftp_h.quit()
		else:
			time.sleep(1)
	except all_errors:
		traceback.print_exc()
		time.sleep(10)
		continue
