#!/usr/bin/env python

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
import os, sys, re, time
from datetime import datetime
from ftplib import FTP
import cPickle
from StringIO import StringIO
import numpy as np
import redis, cv2
from skimage import feature

from email_send import send_email

MODELPATH = 'models/model.svc'
REDIS_LIST = 'garage_files2label'
REDIS_HASH = 'garage_labels'
REDIS_ZLIST = 'garage_labels_ts'

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


def detect_image_label(model, ftp_h, fpath):
	reader = StringIO()
	ftp_h.retrbinary("RETR %s" % fpath, reader.write)
	imgdata = reader.getvalue()
	img_array = np.asarray(bytearray(imgdata), dtype=np.uint8)
	image = cv2.imdecode(img_array, cv2.CV_LOAD_IMAGE_UNCHANGED)
	return (detect_label(model, image), imgdata)

redis = redis.Redis('bbspeaker')
ftp_h = FTP('192.168.1.1')
ftp_h.login('writer', 'pfgbcm')
ftp_h.cwd('rus_hard/garage')
#00D6FB009223(n800sau)_1_20160516142921_30928.jpg
model = cPickle.loads(open(MODELPATH).read())
r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')
for i in range(50):
	fpath = redis.lpop(REDIS_LIST)
	print fpath
	if fpath is None:
		break
	m = r.match(os.path.basename(fpath))
	if m:
		dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
		ts = time.mktime(dt.timetuple())
		try:
			label,imgdata = detect_image_label(model, ftp_h, fpath)
			redis.hset(REDIS_HASH, fpath, label)
			last_ts = redis.zrange(REDIS_ZLIST, -1, -1, withscores=True)
			if last_ts:
				last_label,last_ts = last_ts[0]
				if last_ts < ts and last_label != label:
					msg = 'Detected change at %s from %s to %s (diff=%d)' % (dt.strftime('%d/%m %H:%M:%S'), last_label, label, ts - last_ts)
					print msg
					send_email('itmousecage@gmail.com', 'Detected %s at %s' % (label, dt.strftime('%d/%m %H:%M:%S')), msg, [imgdata])
			redis.zadd(REDIS_ZLIST, label, ts)
		except:
			# return fpath back to redis list
			redis.lpush(REDIS_LIST, fpath)
			raise

ftp_h.quit()
