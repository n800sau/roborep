#!/usr/bin/env python

import os, sys, re
from datetime import datetime
from ftplib import FTP
import cPickle
from StringIO import StringIO
import numpy as np
import redis, cv2
from skimage import feature

MODELPATH = 'models/model.svc'
REDIS_LIST = 'garage_files2label'

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
	img_array = np.asarray(bytearray(reader.getvalue()), dtype=np.uint8)
	image = cv2.imdecode(img_array, cv2.CV_LOAD_IMAGE_UNCHANGED)
	return detect_label(model, image)

redis = redis.Redis('bbspeaker')
ftp_h = FTP('192.168.1.1')
ftp_h.login('writer', 'pfgbcm')
ftp_h.cwd('rus_hard/garage')
#00D6FB009223(n800sau)_1_20160516142921_30928.jpg
model = cPickle.loads(open(MODELPATH).read())
for i in range(20):
	fpath = redis.lpop(REDIS_LIST)
	print fpath
	if fpath is None:
		break
	try:
		label = detect_image_label(model, ftp_h, fpath)
		redis.hset('garage_labels', fpath, label)
	except:
		# return fpath back to redis list
		redis.lpush(REDIS_LIST, fpath)
		raise

ftp_h.quit()
