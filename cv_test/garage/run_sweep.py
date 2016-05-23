#!/usr/bin/env python

import os, sys, re
from datetime import datetime
from ftplib import FTP
import cPickle

import numpy as np
import redis, cv2
from skimage import feature

MODELPATH = 'models/model.svc'

def detect_label(model, image):
	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	hist = feature.hog(image,
		orientations=9,
		pixels_per_cell=[4, 4],
		cells_per_block=[2, 2],
		transform_sqrt=True)
	hist[hist < 0] = 0
	labels = model.predict_proba(hist)
	labels = zip(model.classes_, labels[0])
	labels.sort(key=lambda x: x[1], reverse=True)
	print labels
	return labels[0][0] if abs(labels[0][1] - labels[1][1]) > 0.5 else '_'


def detect_image_label(ftp_h, fname):
	r = redis.Redis()
	model = cPickle.loads(open(MODELPATH).read())
	print 'here0'
	reader = StringIO()
	ftp_h.retrbinary('RETR "%s"' % fname, reader.write)
	print 'here1'
	img_array = np.asarray(bytearray(reader.getvalue()), dtype=np.uint8)
	print 'here2'
	image = cv2.imdecode(img_array)
	print 'here3'
	label = detect_label(model, image)
	r.hset('garage_labels', os.path.basename(fname), label)

r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')

ftp_h = FTP('192.168.1.1')
ftp_h.login('writer', 'pfgbcm')
ftp_h.cwd('rus_hard/garage')
#00D6FB009223(n800sau)_1_20160516142921_30928.jpg
pathlist = ftp_h.nlst()
for srcname in pathlist:
	bname = os.path.basename(srcname)
	m = r.match(bname)
	if m:
		dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
		bdname = dt.strftime('%Y-%m-%d')
		dname = os.path.join(os.path.dirname(srcname), bdname)
		if dname not in pathlist:
			print 'Create directory', dname
			ftp_h.mkd(bdname)
			pathlist.append(dname)
		dstname = os.path.join(dname, bname)
		print bname, '->', dstname
#		detect_image_label(ftp_h, srcname)
		ftp_h.rename(srcname, dstname)

ftp_h.quit()
