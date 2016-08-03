#!/usr/bin/env python

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
import os, sys, re, time, json, traceback
from datetime import datetime
from ftplib import FTP
import cPickle
from StringIO import StringIO
import numpy as np
import redis, cv2
from skimage import feature

from email_send import send_email

MODELLIST = ['inside_empty', 'open_close']

MODELPATH = 'models'
REDIS_INPUT_LIST = 'garage_files2label'
REDIS_FAIL_LIST = 'garage_failed_files'
REDIS_OUTPUT_PREFIX = 'garage_label_'
NO_LABEL = '_'
NO_LABEL_QUEUE_PREFIX = 'no_label_'

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
	rs = labels[0][0] if abs(labels[0][1] - labels[1][1]) > 0.5 else '_'
	print '%s -> %s' % (labels, rs)
	return rs


def detect_image_label(model, ftp_h, fpath):
	t = time.time()
	reader = StringIO()
	ftp_h.retrbinary("RETR %s" % fpath, reader.write)
	imgdata = reader.getvalue()
	img_array = np.asarray(bytearray(imgdata), dtype=np.uint8)
	image = cv2.imdecode(img_array, cv2.CV_LOAD_IMAGE_UNCHANGED)
	rs = (detect_label(model, image), imgdata)
	tdiff = int(time.time() - t)
	print 'image process time: %d:%02d' % (tdiff//60, tdiff%60)
	return rs

#00D6FB009223(n800sau)_1_20160516142921_30928.jpg
failed_file = ''
ftp_h = None
models = None
r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')
try:
	redis = redis.Redis()
	for i in range(5000):
		fpath = redis.lpop(REDIS_INPUT_LIST)
		if fpath is None:
			break
		bname = os.path.basename(fpath)
		m = r.match(bname)
		if m:
			dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
			ts = time.mktime(dt.timetuple())
			try:
				if ftp_h is None:
					ftp_h = FTP('192.168.1.1', timeout=30)
					ftp_h.login('writer', 'pfgbcm')
				if models is None:
					t = time.time()
					models = {}
					for mname in MODELLIST:
						models[mname] = cPickle.loads(open(os.path.join(MODELPATH, mname + '.svc')).read())
#						cPickle.dump(models[mname], open(os.path.join(MODELPATH, mname + '.svc.new'), 'w'), protocol=cPickle.HIGHEST_PROTOCOL)
					tdiff = int(time.time() - t)
					print 'models load time: %d:%02d' % (tdiff//60, tdiff%60)
				msglist = []
				labellist = []
				for mname,model in models.items():
					output_name = REDIS_OUTPUT_PREFIX + mname
					print bname,
					label,imgdata = detect_image_label(model, ftp_h, fpath)
					if label == NO_LABEL:
						queue_pfx = NO_LABEL_QUEUE_PREFIX + mname
						redis.rpush(queue_pfx, bname)
						redis.ltrim(queue_pfx, max(0, redis.llen(queue_pfx) - 100), -1)
					elif label in ('open', 'close'):
						redis.set('gate', json.dumps({'label': label, 'ts': time.time()}))
					if label != NO_LABEL:
						last_rec = redis.lrange(output_name, -1, -1)
						if last_rec:
							last_rec = json.loads(last_rec[0])
							if last_rec['ts'] < ts and last_rec['label'] != label:
								msg = '%s changed at %s from %s to %s (diff=%d), %s' % (mname, dt.strftime('%d/%m %H:%M:%S'), last_rec['label'], label, ts - last_rec['ts'], bname)
								print msg
								msglist.append(msg)
								labellist.append((mname, label))
						else:
							msg = 'Initial at %s %s' % (dt.strftime('%d/%m %H:%M:%S'), label)
							print msg
							msglist.append(msg)
							labellist.append((mname, label))
						print
						redis.rpush(output_name, json.dumps({'label': label, 'ts': ts, 'name': fpath}))
						redis.ltrim(output_name, max(0, redis.llen(output_name) - 100), -1)
				if msglist:
					labellist = [label for mname,label in labellist if label != NO_LABEL]
					if not labellist:
						labellist = ['_']
					send_email('itmousecage@gmail.com', '%s: %s' % (dt.strftime('%H:%M:%S %d/%m'), ','.join(labellist)), '\n'.join(msglist), [imgdata])
			except:
				# return fpath back to redis list
				redis.rpush(REDIS_FAIL_LIST, fpath)
				failed_file = fpath
				raise
except Exception, e:
	send_email('itmousecage@gmail.com', '%s error occured: %s' % (failed_file, str(e)), 'Error details: %s' % traceback.format_exc())
	traceback.print_exc(sys.stderr)

if not ftp_h is None:
	ftp_h.quit()


print 'Finished at %s' % time.strftime('%d/%m %H:%M:%S')
