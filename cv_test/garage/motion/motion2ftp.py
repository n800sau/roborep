#!/usr/bin/env python

import os, sys, time, traceback
import redis
from ftplib import FTP, all_errors


R_QUEUE = 'q.motion_captured'

r = redis.Redis()

while r.llen(R_QUEUE) > 0:
	try:
		ftp_h = FTP('192.168.1.1')
		try:
			ftp_h.login('writer', cred['writer@192.168.1.1'])
			ftp_h.cwd('rus_hard/garage')
			while True:
				fname = r.rpop(R_QUEUE)
				if not fname:
					break
				if os.path.exists(fname):
					f = open(fname, 'rb')
					try:
						session.storbinary('STOR ' + os.path.basename(fname), f)
						f.close()
						os.unlink(fname)
					except:
						r.lpush(R_QUEUE, fname)
						raise
		finally:
			ftp_h.quit()
	except all_errors:
		traceback.print_exc()
		time.sleep(10)
		continue
