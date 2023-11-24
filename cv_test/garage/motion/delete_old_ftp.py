#!/usr/bin/env python

import os, sys, time, traceback, re
sys.path.append(os.path.expanduser('~n800s/private'))
from datetime import datetime
from ftplib import FTP, all_errors
from credential import cred

MAX_DAYS = 180

reg = re.compile('([0-9]{14})-\d+(-\d+)?\.(jpg|avi)')
BASE_DNAME = 'rus_hard/garage'

ftp_h = FTP('192.168.1.1')
try:
	ftp_h.login('garage', cred['garage@192.168.1.1'])
	ftp_h.cwd(BASE_DNAME)
	dirlist = [dname for dname in ftp_h.nlst() if re.match(r'^\d+-\d+-\d+_((pics)|(vids))$', os.path.basename(dname))]
	for dname in dirlist:
		#/rus_hard/garage/2018-11-01_pics
		print os.path.basename(dname)
		try:
			dts = datetime.strptime(os.path.basename(dname).split('_')[0], '%Y-%m-%d')
		except:
			traceback.print_exc()
			continue
		delta = datetime.now() - dts
		if delta.days > MAX_DAYS:
			print os.path.basename(dname), delta.days, 'days old'
			ftp_h.cwd(dname)
			skipped = False
			for fname in ftp_h.nlst():
				if os.path.splitext(fname)[-1] in ('.jpg', '.avi'):
					ftp_h.delete(fname)
				else:
					skipped = True
			ftp_h.cwd('..')
			if not skipped:
				ftp_h.rmd(dname)
				print dname, 'deleted'
finally:
	ftp_h.quit()
