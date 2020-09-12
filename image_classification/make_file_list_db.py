#!/usr/bin/env python3

import os
import lib
import hashlib
import json
import shelve

n = 0
try:
	with shelve.open('imlist.shelve', 'c') as db:

		bpath = os.path.expanduser('~/sshfs/asus/root/g750/hdd750/mydvd')
		print(bpath)
		for i,data in enumerate(lib.dir_images_generator(bpath)):
			fname,exif = data
#			print(fname)
			try:
				if fname not in db:
					f = open(fname,'rb')
					file_md5 = hashlib.md5(f.read()).hexdigest()
					f.close()
					exif['md5sum'] = file_md5
					db[fname] = exif
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

print('Finished')

