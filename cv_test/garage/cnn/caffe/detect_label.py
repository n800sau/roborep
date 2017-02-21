#!/usr/bin/env python

import sys
sys.path.append('../../')

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
import os, sys, re, time, json, traceback
from datetime import datetime
from ftplib import FTP, error_temp
import cPickle
from StringIO import StringIO
import numpy as np
import redis, cv2
from predict import CNNClassificator, arg_parser

from email_send import send_email

REDIS_INPUT_LIST = 'garage_files2label'
REDIS_FAIL_LIST = 'garage_failed_files'
REDIS_OUTPUT_PREFIX = 'garage_label_'
NO_LABEL_QUEUE = 'garage_label_none'

args = vars(arg_parser().parse_args())

def detect_image_label(detector, ftp_h, fpath):
	t = time.time()
	reader = StringIO()
	ftp_h.retrbinary("RETR %s" % fpath, reader.write)
	imgdata = reader.getvalue()
	img_array = np.asarray(bytearray(imgdata), dtype=np.uint8)
	image = cv2.imdecode(img_array, cv2.CV_LOAD_IMAGE_UNCHANGED)
	rs =list(detector.detect(image)) + [imgdata]
	tdiff = int(time.time() - t)
	print 'image process time: %d:%02d' % (tdiff//60, tdiff%60)
	return rs

#00D6FB009223(n800sau)_1_20160516142921_30928.jpg
failed_file = ''
ftp_h = None
detector = None

r = re.compile('^[0-9A-F]+\(.*\)_\d_(\d+)_\d+\.jpg')
try:
	redis = redis.Redis()
	for i in range(100):
		fpath = redis.lpop(REDIS_INPUT_LIST)
		if fpath is None:
			print 'End of files'
			break
		bname = os.path.basename(fpath)
		print 'popped %s' % fpath
		m = r.match(bname)
		if m:
			dt = datetime.strptime(m.groups()[0], '%Y%m%d%H%M%S')
			ts = time.mktime(dt.timetuple())
			try:
				if ftp_h is None:
					ftp_h = FTP('192.168.1.1', timeout=30)
					ftp_h.login('writer', 'pfgbcm')
				if detector is None:
					t = time.time()
					detector = CNNClassificator(args['classlist'], args['snapshot'], args['arch'], args['mean'], args['gpu'])
					tdiff = int(time.time() - t)
					print 'model load time: %d:%02d' % (tdiff//60, tdiff%60)
				label,prob,imgdata = detect_image_label(detector, ftp_h, fpath)
				print('Label: {}, {}%'.format(label, int(prob * 100)))
				output_name = REDIS_OUTPUT_PREFIX + label
				if prob < 0.7:
					queue_pfx = NO_LABEL_QUEUE
					redis.rpush(queue_pfx, bname)
					redis.ltrim(queue_pfx, max(0, redis.llen(queue_pfx) - 100), -1)
				else:
					last_rec = redis.lrange(output_name, -1, -1)
					if last_rec:
						last_rec = json.loads(last_rec[0])
						if last_rec['ts'] < ts and last_rec['label'] != label:
							msg = 'Changed at %s from %s to %s (diff=%d), %s' % (dt.strftime('%d/%m %H:%M:%S'), last_rec['label'], label, ts - last_rec['ts'], bname)
							print msg
							send_email('itmousecage@gmail.com', '%s: %s' % (dt.strftime('%H:%M:%S %d/%m'), label), msg, [imgdata])
					else:
						msg = 'Initial at %s %s' % (dt.strftime('%d/%m %H:%M:%S'), label)
						print msg
						send_email('itmousecage@gmail.com', '%s: %s' % (dt.strftime('%H:%M:%S %d/%m'), label), msg, [imgdata])
					redis.rpush(output_name, json.dumps({'label': label, 'ts': ts, 'name': fpath}))
					redis.ltrim(output_name, max(0, redis.llen(output_name) - 100), -1)
			except:
				# return fpath back to redis list
				redis.rpush(REDIS_FAIL_LIST, fpath)
				failed_file = fpath
				raise
		# for test
#		redis.rpush(REDIS_FAIL_LIST, fpath)
#		break
except Exception, e:
	traceback.print_exc(sys.stderr)
	if not isinstance(e, error_temp):
		send_email('itmousecage@gmail.com', '%s error occured: %s' % (failed_file, str(e)), 'Error details: %s' % traceback.format_exc())

if not ftp_h is None:
	ftp_h.quit()


print 'Finished at %s' % time.strftime('%d/%m %H:%M:%S')
