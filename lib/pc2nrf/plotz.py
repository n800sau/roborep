#!/usr/bin/env python

import csv, time, os, sys, redis
import matplotlib as mpl
mpl.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

inopath = os.path.join(os.path.dirname(__file__), '..', '..', 'ino')
sys.path.append(os.path.join(inopath, 'include', 'swig'))
sys.path.append(os.path.join(inopath, 'lib_py'))

from conf_ino import configure
import libcommon_py as common

# wait before start for 5 mins
time.sleep(300)


NEXT_EVENT_TIMEOUT = 120 #secs

def send_email(msgtxt, fnamelist):
	msg = MIMEMultipart()
	msg['Subject'] = 'door plots %s' % time.strftime('%Y/%m/%d %H:%M')
	msg['From'] = 'itmousecage@gmail.com'
	msg['To'] = 'n800sau@gmail.com'
	msg.attach(MIMEText(msgtxt, 'plain', 'utf-8'))
	for fname in fnamelist:
		msg.attach(MIMEImage(file(fname).read()))

	smtpserver = smtplib.SMTP("smtp.gmail.com",587)
	smtpserver.ehlo()
	smtpserver.starttls()
	smtpserver.ehlo
	smtpserver.login('itmousecage@gmail.com', 'rktnrf000')
	smtpserver.sendmail(msg['From'], msg['To'], msg.as_string())
	smtpserver.close()

def draw_plot(ldict):

	fig = plt.figure()
	fig.suptitle('Data count %d' % len(ldict), fontsize=12)

	if data_x:
		ax1 = fig.add_subplot(221)
		ax1.set_title("X")
		ax1.set_xlabel('time')
		ax1.set_ylabel('value')
		ax1.plot(data_time, data_x, color='r', label='X')
		ax1.legend()
		plt.gcf().autofmt_xdate(bottom=0.2, rotation=30, ha='right')

	if data_y:
		ax2 = fig.add_subplot(222)
		ax2.set_title("Y")
		ax2.set_xlabel('time')
		ax2.set_ylabel('value')
		ax2.plot(data_time, data_y, color='g', label='Y')
		ax2.legend()
		plt.gcf().autofmt_xdate(bottom=0.2, rotation=30, ha='right')

	if data_z:
		ax3 = fig.add_subplot(223)
		ax3.set_title("X")
		ax3.set_xlabel('time')
		ax3.set_ylabel('value')
		ax3.plot(data_time, data_z, color='b', label='Z')
		ax3.legend()
		plt.gcf().autofmt_xdate(bottom=0.2, rotation=30, ha='right')


cfgobj = configure(os.path.dirname(__file__))
cfg = cfgobj.as_dict('serial2redis')
params = {'host': cfg.get('redis_host', 'localhost')}
redis_port = cfg.get('redis_port', None)
if redis_port:
	params['port'] = redis_port

r = redis.Redis(**params)

timeout = timedelta(seconds=NEXT_EVENT_TIMEOUT)
f = file(os.path.join(os.path.dirname(__file__), 'data_4.csv'))
vf = csv.reader(f)
data_time = []
data_x = []
data_y = []
data_z = []
last_t = None
for row in vf:
	ldict = dict(zip(row[::2], row[1::2]))
	t = datetime.fromtimestamp(float(ldict['secs']) + float(ldict['moffset'])/1000)
	if int(ldict['type']) == common.PL_ACC:
		if last_t is None or last_t + timeout < t or last_t > t:
			# draw plot and start next plot
			if (data_x and max(data_x)-min(data_x) > 100) or (data_y and max(data_y)-min(data_y) > 100) or (data_z and max(data_z)-min(data_z) > 100):
				# draw plot
				bname = 'plot_%s.png' % last_t.strftime('%Y.%m.%d-%H:%M')
				if not r.sismember('s.files', bname):
					draw_plot(ldict)
					fname = os.path.join('/var/www/http/atad', bname)
					plt.savefig(fname, bbox_inches='tight')
					print fname, 'saved'
					send_email('door plots', [fname])
					r.sadd('s.files', bname)
					print bname, 'sent'
				#plt.show()
			data_x = []
			data_y = []
			data_z = []
			data_time = []
		else:
			data_time.append(t)
			data_x.append(float(ldict['acc_x']))
			data_y.append(float(ldict['acc_y']))
			data_z.append(float(ldict['acc_z']))
	last_t = t

#data = np.genfromtxt('q.R.csv', delimiter=',', usecols = (1, 7), names=['x', 'y'])



