#!/usr/bin/env python3

import os
import lib
import hashlib
import json
import dbm

n = 0
try:
	with dbm.open('imlist.db', 'c') as db:

		bpath = os.path.expanduser('~/sshfs/asus/root/g750/hdd750/mydvd')
		print(bpath)
		for i,fname in enumerate(lib.dir_images_generator(bpath)):
#			print(fname)
			try:
				if fname not in db:
					f = open(fname,'rb')
					file_md5 = hashlib.md5(f.read()).hexdigest()
					f.close()
					db[fname] = file_md5
					# sync after every 10th file
					if (i+1)%10 == 0:
						db.sync()
				n += 1
			except Exception as e:
				print('%s' % e)
				continue
except:
	print('Error on %d file' % n)
	raise




